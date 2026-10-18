/* (c) DDNet developers. See licence.txt in the root of the distribution for more information. */
#ifndef GAME_CLIENT_COMPONENTS_AUTOPILOT_H
#define GAME_CLIENT_COMPONENTS_AUTOPILOT_H

#include <base/vmath.h>

#include <generated/protocol.h>

#include <game/client/component.h>
#include <game/client/prediction/gameworld.h>

#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <utility>
#include <vector>

class CCharacter;
class CEntity;

/**
 * Autopilot that takes over the local tee and races it to the finish.
 *
 * Planning uses the client's own prediction world, so every candidate is
 * simulated tick by tick with the real game physics (hook, jumps, speedups,
 * freeze, explosions, laser and shotgun bounces). A beam search over short
 * "macro actions" (direction, jump, hook at an angle, fire a weapon at an
 * angle) is scored by a distance field that leads to the finish tiles around
 * freeze and death tiles. The first macro of the best plan is committed, the
 * rest warm-starts the next search. The search is incremental: it runs under
 * a per-frame time budget while previously committed inputs are being played.
 *
 * The autopilot only works on local/LAN servers or in /practice mode so it can
 * never be used to set ranks.
 */
class CAutopilot : public CComponent
{
public:
	CAutopilot();
	~CAutopilot() override;
	int Sizeof() const override { return sizeof(*this); }

	void OnMapLoad() override;
	void OnReset() override;
	void OnUpdate() override;
	void OnRender() override;

	bool IsActive() const { return m_Active; }
	// replaces the movement part of the input sent for the given tick
	void OverrideInput(int Tick, CNetObj_PlayerInput *pInput);

private:
	enum EHookMode
	{
		HOOKMODE_RELEASE = 0,
		HOOKMODE_HOLD,
		HOOKMODE_NEW,
	};

	enum EOutcome
	{
		OUTCOME_NONE = 0,
		OUTCOME_FINISH,
		OUTCOME_TELEPORT,
		OUTCOME_FROZEN,
		OUTCOME_DEAD,
	};

	struct SAction
	{
		int m_Direction = 0;
		bool m_Jump = false;
		int m_HookMode = HOOKMODE_RELEASE;
		float m_Angle = 0.0f;
		int m_Weapon = -1; // weapon to fire, -1 = none
	};

	struct SNode
	{
		std::unique_ptr<CGameWorld> m_pWorld;
		std::vector<SAction> m_vPlan;
		CNetObj_PlayerInput m_LastInput;
		float m_Cost = 0.0f;
		int m_Elapsed = 0;
		bool m_Warm = false; // follows the previous best plan
	};

	struct SSimResult
	{
		int m_Outcome = OUTCOME_NONE;
		int m_Ticks = 0;
		float m_TeleDist = 0.0f;
	};

	// map analysis
	enum
	{
		TILEFLAG_SOLID = 1 << 0,
		TILEFLAG_FREEZE = 1 << 1,
		TILEFLAG_DEATH = 1 << 2,
		TILEFLAG_FINISH = 1 << 3,
		TILEFLAG_TELEIN = 1 << 4,
		TILEFLAG_BADTELE = 1 << 5,
		TILEFLAG_NEARHAZARD = 1 << 6,
	};
	int m_Width = 0;
	int m_Height = 0;
	std::vector<uint8_t> m_vTileFlags;
	std::vector<float> m_vDist;
	bool m_HasFinish = false;

	void BuildDistanceField();
	float TileDist(int x, int y) const;
	int TileFlagsAt(vec2 Pos) const;
	float DistanceAt(vec2 Pos) const;
	float GuideAngle(vec2 Pos) const;

	// simulation
	void DetachedCopy(CGameWorld *pTo, CGameWorld *pFrom);
	std::unique_ptr<CGameWorld> AcquireWorld();
	void ReleaseWorld(std::unique_ptr<CGameWorld> pWorld);
	bool SimulateTick(CGameWorld *pWorld, int Tick, const CNetObj_PlayerInput &Input) const;
	CNetObj_PlayerInput BuildInput(const SAction &Action, int SubTick, const CNetObj_PlayerInput &LastInput, const CCharacter *pChar) const;
	static CNetObj_PlayerInput NeutralInput(const CNetObj_PlayerInput &LastInput);
	SSimResult SimulateAction(CGameWorld *pWorld, int StartTick, const SAction &Action, CNetObj_PlayerInput &LastInput, std::vector<CNetObj_PlayerInput> *pvInputs, std::vector<vec2> *pvPath) const;
	int CheckOutcome(const CCharacter *pChar, vec2 PrevPos, float &TeleDist) const;
	float StateCost(const CCharacter *pChar, int Elapsed) const;
	float TerminalCost(const SSimResult &Result, const CCharacter *pChar, int Elapsed) const;
	static uint64_t StateKey(const CCharacter *pChar);

	// search
	bool IsAllowed() const;
	void Start();
	void Stop(const char *pReason);
	void ResetPlan();
	void ReleaseNodes(std::vector<SNode> &vNodes);
	bool StartSearch(int SearchTick);
	void StepSearch(int64_t Deadline);
	void ExpandNode(SNode &Parent);
	void SampleActions(const SNode &Parent, const CCharacter *pChar, bool &HasWarm);
	void SelectNextBeam();
	void FinalizeSearch();
	float RandomFloat();

	bool m_Active = false;
	int m_LocalId = -1;
	int m_PrevMouseInputType = 0;
	bool m_HaveLastActualPos = false;
	vec2 m_LastActualPos = vec2(0.0f, 0.0f);

	// inputs that are decided, by tick
	std::map<int, CNetObj_PlayerInput> m_Committed;
	int m_CommitEnd = -1;
	std::vector<SAction> m_vWarmPlan;
	int m_ExpectedTick = -1;
	vec2 m_ExpectedPos = vec2(0.0f, 0.0f);

	// current search
	bool m_Searching = false;
	int m_SearchTick = 0;
	int m_Step = 3;
	int m_MaxDepth = 10;
	int m_BeamWidth = 24;
	int m_Samples = 32;
	int m_Depth = 0;
	size_t m_ParentIndex = 0;
	bool m_RootFrozen = false;
	std::unique_ptr<CGameWorld> m_pRoot;
	CNetObj_PlayerInput m_RootInput;
	std::vector<SNode> m_vBeam;
	std::vector<SNode> m_vChildren;
	std::vector<SAction> m_vBestTerminalPlan;
	float m_BestTerminalCost = 0.0f;
	int m_BestTerminalOutcome = OUTCOME_NONE;
	std::vector<SAction> m_vActions;

	std::vector<std::unique_ptr<CGameWorld>> m_vpWorldPool;
	std::vector<std::pair<CEntity *, CEntity *>> m_vSavedLinks;
	std::minstd_rand m_Rng;

	// feedback
	std::vector<vec2> m_vDebugPath;
	int m_PlanOutcome = OUTCOME_NONE;
};

#endif

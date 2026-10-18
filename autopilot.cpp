/* (c) DDNet developers. See licence.txt in the root of the distribution for more information. */
#include "autopilot.h"

#include <base/math.h>
#include <base/mem.h>
#include <base/net.h>
#include <base/str.h>
#include <base/time.h>

#include <engine/client.h>
#include <engine/graphics.h>
#include <engine/shared/config.h>
#include <engine/textrender.h>

#include <generated/protocol.h>

#include <game/client/components/controls.h>
#include <game/client/gameclient.h>
#include <game/client/prediction/entities/character.h>
#include <game/client/prediction/entity.h>
#include <game/collision.h>
#include <game/gamecore.h>
#include <game/mapitems.h>

#include <algorithm>
#include <cmath>
#include <queue>
#include <unordered_set>

namespace
{
constexpr float INF_DIST = 1e9f;
// cost of a path that cannot reach the finish, still finite so such states stay comparable
constexpr float UNREACHABLE_COST = 5e7f;
constexpr float FREEZE_PENALTY = 1e5f;
constexpr float DEATH_PENALTY = 2e5f;
constexpr float FINISH_REWARD = 1e6f;
// tile cost multipliers used by the distance field
constexpr float FREEZE_TILE_COST = 40.0f;
constexpr float NEAR_HAZARD_COST = 2.0f;
// px per tick used to trade remaining distance against elapsed time
constexpr float REF_SPEED = 12.0f;
// how many ticks of current speed along the route count as progress
constexpr float SPEED_LOOKAHEAD = 6.0f;
constexpr float AIM_DISTANCE = 256.0f;
// predicted and real position drift that invalidates the committed inputs
constexpr float DESYNC_DISTANCE = 40.0f;

bool IsFreezeTile(int Tile)
{
	return Tile == TILE_FREEZE || Tile == TILE_DFREEZE || Tile == TILE_LFREEZE;
}
}

CAutopilot::CAutopilot() :
	m_Rng((unsigned)time_get())
{
	mem_zero(&m_RootInput, sizeof(m_RootInput));
}

CAutopilot::~CAutopilot() = default;

void CAutopilot::OnMapLoad()
{
	BuildDistanceField();
}

void CAutopilot::OnReset()
{
	ResetPlan();
	m_Active = false;
	m_LocalId = -1;
	m_HaveLastActualPos = false;
	m_pRoot.reset();
	m_vpWorldPool.clear();
}

// ---------------------------------------------------------------------------
// map analysis
// ---------------------------------------------------------------------------

void CAutopilot::BuildDistanceField()
{
	CCollision *pCollision = GameClient()->Collision();
	m_Width = pCollision->GetWidth();
	m_Height = pCollision->GetHeight();
	const int NumTiles = std::max(m_Width * m_Height, 0);
	m_vTileFlags.assign(NumTiles, 0);
	m_vDist.assign(NumTiles, INF_DIST);
	m_HasFinish = false;
	if(NumTiles == 0)
		return;

	const CTeleTile *pTele = pCollision->TeleLayer();
	std::map<int, std::vector<int>> TeleInTiles;
	std::vector<int> vTeleOutNumber(NumTiles, 0);
	for(int i = 0; i < NumTiles; i++)
	{
		const int Game = pCollision->GetTileIndex(i);
		const int Front = pCollision->GetFrontTileIndex(i);
		const int Switch = pCollision->GetSwitchType(i);
		uint8_t Flags = 0;
		if(Game == TILE_SOLID || Game == TILE_NOHOOK)
			Flags |= TILEFLAG_SOLID;
		if(Game == TILE_DEATH || Front == TILE_DEATH)
			Flags |= TILEFLAG_DEATH;
		if(IsFreezeTile(Game) || IsFreezeTile(Front) || IsFreezeTile(Switch))
			Flags |= TILEFLAG_FREEZE;
		if(Game == TILE_FINISH || Front == TILE_FINISH)
			Flags |= TILEFLAG_FINISH;
		if(pTele)
		{
			const int Type = pTele[i].m_Type;
			const int Number = pTele[i].m_Number;
			if((Type == TILE_TELEIN || Type == TILE_TELEINEVIL) && Number > 0)
			{
				Flags |= TILEFLAG_TELEIN;
				TeleInTiles[Number].push_back(i);
			}
			else if(Type == TILE_TELECHECKIN || Type == TILE_TELECHECKINEVIL)
			{
				// sends the tee back to its last checkpoint, never part of a good route
				Flags |= TILEFLAG_BADTELE;
			}
			else if(Type == TILE_TELEOUT && Number > 0)
			{
				vTeleOutNumber[i] = Number;
			}
		}
		m_vTileFlags[i] = Flags;
	}

	// keep some distance to freeze and death tiles
	for(int y = 0; y < m_Height; y++)
	{
		for(int x = 0; x < m_Width; x++)
		{
			uint8_t &Flags = m_vTileFlags[y * m_Width + x];
			if(Flags & (TILEFLAG_SOLID | TILEFLAG_FREEZE | TILEFLAG_DEATH))
				continue;
			for(int dy = -1; dy <= 1 && !(Flags & TILEFLAG_NEARHAZARD); dy++)
			{
				for(int dx = -1; dx <= 1; dx++)
				{
					const int nx = x + dx, ny = y + dy;
					if(nx < 0 || ny < 0 || nx >= m_Width || ny >= m_Height)
						continue;
					if(m_vTileFlags[ny * m_Width + nx] & (TILEFLAG_FREEZE | TILEFLAG_DEATH))
					{
						Flags |= TILEFLAG_NEARHAZARD;
						break;
					}
				}
			}
		}
	}

	// dijkstra from all finish tiles, walking the route backwards
	using SItem = std::pair<float, int>;
	std::priority_queue<SItem, std::vector<SItem>, std::greater<>> Queue;
	for(int i = 0; i < NumTiles; i++)
	{
		if((m_vTileFlags[i] & TILEFLAG_FINISH) && !(m_vTileFlags[i] & TILEFLAG_SOLID))
		{
			m_vDist[i] = 0.0f;
			Queue.emplace(0.0f, i);
			m_HasFinish = true;
		}
	}

	auto Relax = [&](int Index, float Dist) {
		if(Dist < m_vDist[Index])
		{
			m_vDist[Index] = Dist;
			Queue.emplace(Dist, Index);
		}
	};
	auto IsSolid = [&](int x, int y) {
		return x < 0 || y < 0 || x >= m_Width || y >= m_Height || (m_vTileFlags[y * m_Width + x] & TILEFLAG_SOLID);
	};

	while(!Queue.empty())
	{
		const auto [Dist, Index] = Queue.top();
		Queue.pop();
		if(Dist > m_vDist[Index])
			continue;

		// entering a teleporter leading here is as far from the finish as this tile
		if(vTeleOutNumber[Index])
		{
			const auto It = TeleInTiles.find(vTeleOutNumber[Index]);
			if(It != TeleInTiles.end())
				for(int In : It->second)
					Relax(In, Dist);
		}

		const int x = Index % m_Width;
		const int y = Index / m_Width;
		for(int dy = -1; dy <= 1; dy++)
		{
			for(int dx = -1; dx <= 1; dx++)
			{
				if(dx == 0 && dy == 0)
					continue;
				const int nx = x + dx, ny = y + dy;
				if(IsSolid(nx, ny))
					continue;
				const bool Diagonal = dx != 0 && dy != 0;
				if(Diagonal && (IsSolid(x + dx, y) || IsSolid(x, y + dy)))
					continue;
				const int Neighbor = ny * m_Width + nx;
				const uint8_t Flags = m_vTileFlags[Neighbor];
				// teleporters are only reached through their exits, walking into one never continues past it
				if(Flags & (TILEFLAG_DEATH | TILEFLAG_BADTELE | TILEFLAG_TELEIN))
					continue;
				float Cost = Diagonal ? 45.25f : 32.0f;
				if(Flags & TILEFLAG_FREEZE)
					Cost *= FREEZE_TILE_COST;
				else if(Flags & TILEFLAG_NEARHAZARD)
					Cost *= NEAR_HAZARD_COST;
				Relax(Neighbor, Dist + Cost);
			}
		}
	}
}

float CAutopilot::TileDist(int x, int y) const
{
	if(x < 0 || y < 0 || x >= m_Width || y >= m_Height)
		return INF_DIST;
	return m_vDist[y * m_Width + x];
}

int CAutopilot::TileFlagsAt(vec2 Pos) const
{
	const int x = (int)std::floor(Pos.x / 32.0f);
	const int y = (int)std::floor(Pos.y / 32.0f);
	if(x < 0 || y < 0 || x >= m_Width || y >= m_Height)
		return 0;
	return m_vTileFlags[y * m_Width + x];
}

float CAutopilot::DistanceAt(vec2 Pos) const
{
	if(m_vDist.empty())
		return INF_DIST;
	const float Center = TileDist((int)std::floor(Pos.x / 32.0f), (int)std::floor(Pos.y / 32.0f));
	if(Center >= INF_DIST)
		return INF_DIST;

	// bilinear between tile centers, walls count as slightly worse than the own tile
	const float fx = Pos.x / 32.0f - 0.5f;
	const float fy = Pos.y / 32.0f - 0.5f;
	const int x0 = (int)std::floor(fx);
	const int y0 = (int)std::floor(fy);
	const float tx = fx - x0;
	const float ty = fy - y0;
	auto Corner = [&](int x, int y) {
		const float Dist = TileDist(x, y);
		return Dist >= INF_DIST ? Center + 32.0f : Dist;
	};
	const float Top = mix(Corner(x0, y0), Corner(x0 + 1, y0), tx);
	const float Bottom = mix(Corner(x0, y0 + 1), Corner(x0 + 1, y0 + 1), tx);
	return mix(Top, Bottom, ty);
}

float CAutopilot::GuideAngle(vec2 Pos) const
{
	// follow the steepest descent of the distance field for a few tiles
	vec2 Target = Pos;
	float Best = DistanceAt(Pos);
	if(Best >= INF_DIST)
		return -pi / 2.0f;
	for(int Step = 0; Step < 8; Step++)
	{
		vec2 BestNext = Target;
		float BestNextDist = Best;
		for(int i = 0; i < 8; i++)
		{
			const vec2 Candidate = Target + direction(i * pi / 4.0f) * 32.0f;
			const float Dist = DistanceAt(Candidate);
			if(Dist < BestNextDist)
			{
				BestNextDist = Dist;
				BestNext = Candidate;
			}
		}
		if(BestNextDist >= Best)
			break;
		Target = BestNext;
		Best = BestNextDist;
	}
	if(distance(Target, Pos) < 1.0f)
		return -pi / 2.0f;
	return angle(Target - Pos);
}

// ---------------------------------------------------------------------------
// simulation
// ---------------------------------------------------------------------------

void CAutopilot::DetachedCopy(CGameWorld *pTo, CGameWorld *pFrom)
{
	// CopyWorld links the copy to its source so the client can track predicted
	// entities. The autopilot's worlds are private scratch copies: restore the
	// source's links and cut the copy loose so neither side sees the other.
	CGameWorld *pOldChild = pFrom->m_pChild;
	const bool OldChildValid = pOldChild ? pOldChild->m_IsValidCopy : false;
	m_vSavedLinks.clear();
	for(int Type = 0; Type < CGameWorld::NUM_ENTTYPES; Type++)
		for(CEntity *pEnt = pFrom->FindFirst(Type); pEnt; pEnt = pEnt->TypeNext())
			m_vSavedLinks.emplace_back(pEnt, pEnt->m_pChild);

	pTo->CopyWorld(pFrom);

	for(auto &[pEnt, pChild] : m_vSavedLinks)
		pEnt->m_pChild = pChild;
	pFrom->m_pChild = pOldChild;
	if(pOldChild)
		pOldChild->m_IsValidCopy = OldChildValid;

	pTo->m_pParent = nullptr;
	pTo->m_pChild = nullptr;
	for(int Type = 0; Type < CGameWorld::NUM_ENTTYPES; Type++)
	{
		for(CEntity *pEnt = pTo->FindFirst(Type); pEnt; pEnt = pEnt->TypeNext())
		{
			pEnt->m_pParent = nullptr;
			pEnt->m_pChild = nullptr;
		}
	}
}

std::unique_ptr<CGameWorld> CAutopilot::AcquireWorld()
{
	if(m_vpWorldPool.empty())
		return std::make_unique<CGameWorld>();
	std::unique_ptr<CGameWorld> pWorld = std::move(m_vpWorldPool.back());
	m_vpWorldPool.pop_back();
	return pWorld;
}

void CAutopilot::ReleaseWorld(std::unique_ptr<CGameWorld> pWorld)
{
	if(pWorld)
		m_vpWorldPool.push_back(std::move(pWorld));
}

bool CAutopilot::SimulateTick(CGameWorld *pWorld, int Tick, const CNetObj_PlayerInput &Input) const
{
	// same order as CGameClient::OnPredict
	CCharacter *pChar = pWorld->GetCharacterById(m_LocalId);
	if(!pChar)
		return false;
	pChar->OnDirectInput(&Input);
	pWorld->m_GameTick = Tick;
	pChar->OnPredictedInput(&Input);
	pWorld->Tick();
	return pWorld->GetCharacterById(m_LocalId) != nullptr;
}

CNetObj_PlayerInput CAutopilot::NeutralInput(const CNetObj_PlayerInput &LastInput)
{
	CNetObj_PlayerInput Input = LastInput;
	Input.m_Direction = 0;
	Input.m_Jump = 0;
	if(Input.m_Fire & 1)
		Input.m_Fire = (Input.m_Fire + 1) & INPUT_STATE_MASK;
	return Input;
}

CNetObj_PlayerInput CAutopilot::BuildInput(const SAction &Action, int SubTick, const CNetObj_PlayerInput &LastInput, const CCharacter *pChar) const
{
	CNetObj_PlayerInput Input = LastInput;
	Input.m_PlayerFlags = PLAYERFLAG_PLAYING | PLAYERFLAG_INPUT_ABSOLUTE;
	Input.m_Direction = Action.m_Direction;

	const vec2 Aim = direction(Action.m_Angle) * AIM_DISTANCE;
	Input.m_TargetX = round_to_int(Aim.x);
	Input.m_TargetY = round_to_int(Aim.y);
	if(Input.m_TargetX == 0 && Input.m_TargetY == 0)
		Input.m_TargetX = 1;

	// jumps trigger on press, so press on the first tick and release afterwards
	Input.m_Jump = Action.m_Jump && SubTick == 0;

	switch(Action.m_HookMode)
	{
	case HOOKMODE_RELEASE:
		Input.m_Hook = 0;
		break;
	case HOOKMODE_HOLD:
		Input.m_Hook = 1;
		break;
	case HOOKMODE_NEW:
		// a new hook needs the button released for a tick first
		Input.m_Hook = (SubTick == 0 && LastInput.m_Hook) ? 0 : 1;
		break;
	}

	bool Fire = false;
	if(Action.m_Weapon >= 0)
	{
		Input.m_WantedWeapon = Action.m_Weapon + 1;
		// give the weapon switch a tick if needed
		Fire = pChar->GetActiveWeapon() == Action.m_Weapon || SubTick >= 1;
	}
	if(((Input.m_Fire & 1) != 0) != Fire)
		Input.m_Fire = (Input.m_Fire + 1) & INPUT_STATE_MASK;
	return Input;
}

CAutopilot::SSimResult CAutopilot::SimulateAction(CGameWorld *pWorld, int StartTick, const SAction &Action, CNetObj_PlayerInput &LastInput, std::vector<CNetObj_PlayerInput> *pvInputs, std::vector<vec2> *pvPath) const
{
	SSimResult Result;
	for(int SubTick = 0; SubTick < m_Step; SubTick++)
	{
		const CCharacter *pChar = pWorld->GetCharacterById(m_LocalId);
		if(!pChar)
		{
			Result.m_Outcome = OUTCOME_DEAD;
			return Result;
		}
		const vec2 PrevPos = pChar->Core()->m_Pos;
		const CNetObj_PlayerInput Input = BuildInput(Action, SubTick, LastInput, pChar);
		SimulateTick(pWorld, StartTick + SubTick, Input);
		LastInput = Input;
		Result.m_Ticks = SubTick + 1;
		if(pvInputs)
			pvInputs->push_back(Input);

		pChar = pWorld->GetCharacterById(m_LocalId);
		if(pvPath && pChar)
			pvPath->push_back(pChar->Core()->m_Pos);
		Result.m_Outcome = CheckOutcome(pChar, PrevPos, Result.m_TeleDist);
		if(Result.m_Outcome != OUTCOME_NONE)
			return Result;
	}
	return Result;
}

int CAutopilot::CheckOutcome(const CCharacter *pChar, vec2 PrevPos, float &TeleDist) const
{
	if(!pChar)
		return OUTCOME_DEAD;
	const CCharacterCore *pCore = pChar->Core();
	const vec2 Pos = pCore->m_Pos;
	if(Pos.x < 0.0f || Pos.y < 0.0f || Pos.x >= m_Width * 32.0f || Pos.y >= m_Height * 32.0f)
		return OUTCOME_DEAD;

	// fast tees cross several tiles per tick, check the whole way
	const int Steps = std::clamp((int)std::ceil(distance(PrevPos, Pos) / 12.0f), 1, 64);
	for(int s = 1; s <= Steps; s++)
	{
		const vec2 Point = mix(PrevPos, Pos, s / (float)Steps);
		const int Flags = TileFlagsAt(Point);
		if(Flags & TILEFLAG_FINISH)
			return OUTCOME_FINISH;
		if(Flags & TILEFLAG_DEATH)
			return OUTCOME_DEAD;
		if(Flags & TILEFLAG_TELEIN)
		{
			TeleDist = TileDist((int)std::floor(Point.x / 32.0f), (int)std::floor(Point.y / 32.0f));
			return OUTCOME_TELEPORT;
		}
		if(Flags & TILEFLAG_BADTELE)
			return OUTCOME_FROZEN;
		if(!m_RootFrozen && (Flags & TILEFLAG_FREEZE))
			return OUTCOME_FROZEN;
	}
	if(!m_RootFrozen && (pChar->m_FreezeTime > 0 || pCore->m_IsInFreeze || pCore->m_DeepFrozen))
		return OUTCOME_FROZEN;
	return OUTCOME_NONE;
}

float CAutopilot::StateCost(const CCharacter *pChar, int Elapsed) const
{
	const vec2 Pos = pChar->Core()->m_Pos;
	const vec2 Vel = pChar->Core()->m_Vel;
	const float Dist = DistanceAt(Pos);
	if(Dist >= INF_DIST)
		return UNREACHABLE_COST + REF_SPEED * Elapsed;

	// reward speed along the route, it pays off after the planning horizon
	float Progress = 0.0f;
	const float Left = DistanceAt(Pos - vec2(8.0f, 0.0f));
	const float Right = DistanceAt(Pos + vec2(8.0f, 0.0f));
	const float Up = DistanceAt(Pos - vec2(0.0f, 8.0f));
	const float Down = DistanceAt(Pos + vec2(0.0f, 8.0f));
	if(std::max({Left, Right, Up, Down}) < INF_DIST)
	{
		const vec2 Gradient((Right - Left) / 16.0f, (Down - Up) / 16.0f);
		Progress = std::clamp(-dot(Vel, Gradient), -40.0f, 40.0f);
	}

	float Cost = Dist - SPEED_LOOKAHEAD * Progress + REF_SPEED * Elapsed;
	if(m_RootFrozen && pChar->Core()->m_IsInFreeze)
		Cost += 2000.0f;
	return Cost;
}

float CAutopilot::TerminalCost(const SSimResult &Result, const CCharacter *pChar, int Elapsed) const
{
	float Dist = pChar ? DistanceAt(pChar->Core()->m_Pos) : UNREACHABLE_COST;
	Dist = std::min(Dist, UNREACHABLE_COST);
	switch(Result.m_Outcome)
	{
	case OUTCOME_FINISH:
		return -FINISH_REWARD + REF_SPEED * Elapsed;
	case OUTCOME_TELEPORT:
		return std::min(Result.m_TeleDist, UNREACHABLE_COST) + REF_SPEED * Elapsed;
	case OUTCOME_FROZEN:
		// if it can't be avoided, freeze as late as possible
		return FREEZE_PENALTY + Dist - 50.0f * Elapsed;
	default:
		return DEATH_PENALTY + Dist - 50.0f * Elapsed;
	}
}

uint64_t CAutopilot::StateKey(const CCharacter *pChar)
{
	const CCharacterCore *pCore = pChar->Core();
	auto Quantize = [](float Value, float Scale) {
		return (uint64_t)(uint16_t)(int16_t)std::clamp(round_to_int(Value / Scale), -32000, 32000);
	};
	uint64_t Key = Quantize(pCore->m_Pos.x, 4.0f);
	Key |= Quantize(pCore->m_Pos.y, 4.0f) << 16;
	Key |= Quantize(pCore->m_Vel.x, 1.5f) << 32;
	Key |= (Quantize(pCore->m_Vel.y, 1.5f) & 0x3fff) << 46;
	Key |= (uint64_t)(pCore->m_HookState == HOOK_GRABBED) << 60;
	Key |= (uint64_t)(pCore->m_Jumped & 3) << 61;
	Key |= (uint64_t)(pCore->m_HookState == HOOK_FLYING) << 63;
	return Key;
}

// ---------------------------------------------------------------------------
// search
// ---------------------------------------------------------------------------

float CAutopilot::RandomFloat()
{
	return std::uniform_real_distribution<float>(0.0f, 1.0f)(m_Rng);
}

bool CAutopilot::IsAllowed() const
{
	if(Client()->State() != IClient::STATE_ONLINE)
		return false;

	// local and LAN servers, e.g. the one started from the menu
	const NETADDR &Addr = Client()->ServerAddress();
	if((Addr.type & (NETTYPE_IPV4 | NETTYPE_IPV6 | NETTYPE_WEBSOCKET_IPV4 | NETTYPE_WEBSOCKET_IPV6)) != 0 && net_addr_is_local(&Addr))
		return true;

	// practice mode never saves ranks
	const int LocalId = GameClient()->m_Snap.m_LocalClientId;
	if(LocalId >= 0)
	{
		const auto &Character = GameClient()->m_Snap.m_aCharacters[LocalId];
		if(Character.m_HasExtendedDisplayInfo && (Character.m_ExtendedData.m_Flags & CHARACTERFLAG_PRACTICE_MODE))
			return true;
	}
	return false;
}

void CAutopilot::Start()
{
	m_Active = true;
	m_PrevMouseInputType = (int)GameClient()->m_Controls.m_aMouseInputType[g_Config.m_ClDummy];
	m_HaveLastActualPos = false;
	ResetPlan();
	if(m_vDist.empty())
		BuildDistanceField();

	if(!m_HasFinish)
		GameClient()->Echo("Autopilot engaged, but this map has no finish tiles");
	else if(GameClient()->m_Snap.m_pLocalCharacter && DistanceAt(vec2(GameClient()->m_Snap.m_pLocalCharacter->m_X, GameClient()->m_Snap.m_pLocalCharacter->m_Y)) >= INF_DIST)
		GameClient()->Echo("Autopilot engaged, but no route to the finish was found from here");
	else
		GameClient()->Echo("Autopilot engaged");
}

void CAutopilot::Stop(const char *pReason)
{
	if(!m_Active)
		return;
	m_Active = false;
	ResetPlan();

	// hand the tee back with all buttons released
	CControls &Controls = GameClient()->m_Controls;
	const int Dummy = g_Config.m_ClDummy;
	Controls.ResetInput(Dummy);
	Controls.m_aInputData[Dummy].m_Hook = 0;
	Controls.m_aMouseInputType[Dummy] = (CControls::EMouseInputType)m_PrevMouseInputType;
	if(pReason)
		GameClient()->Echo(pReason);
}

void CAutopilot::ReleaseNodes(std::vector<SNode> &vNodes)
{
	for(SNode &Node : vNodes)
		ReleaseWorld(std::move(Node.m_pWorld));
	vNodes.clear();
}

void CAutopilot::ResetPlan()
{
	ReleaseNodes(m_vBeam);
	ReleaseNodes(m_vChildren);
	m_Committed.clear();
	m_CommitEnd = -1;
	m_vWarmPlan.clear();
	m_vBestTerminalPlan.clear();
	m_ExpectedTick = -1;
	m_Searching = false;
	m_vDebugPath.clear();
	m_PlanOutcome = OUTCOME_NONE;
}

bool CAutopilot::StartSearch(int SearchTick)
{
	ReleaseNodes(m_vBeam);
	ReleaseNodes(m_vChildren);
	m_vBestTerminalPlan.clear();
	m_BestTerminalCost = INF_DIST;
	m_BestTerminalOutcome = OUTCOME_NONE;
	m_Searching = false;

	m_Step = g_Config.m_ClAutopilotStep;
	m_MaxDepth = g_Config.m_ClAutopilotDepth;
	m_BeamWidth = g_Config.m_ClAutopilotBeam;
	m_Samples = g_Config.m_ClAutopilotSamples;

	// start from the latest snapshot, alone in the world
	if(!m_pRoot)
		m_pRoot = AcquireWorld();
	DetachedCopy(m_pRoot.get(), &GameClient()->m_GameWorld);
	m_pRoot->m_PredictedEvents.clear();
	for(int i = 0; i < MAX_CLIENTS; i++)
		if(i != m_LocalId)
			if(CCharacter *pChar = m_pRoot->GetCharacterById(i))
				pChar->Destroy();
	if(!m_pRoot->GetCharacterById(m_LocalId))
		return false;

	// replay what was sent and what is decided up to the tick the plan starts at
	const int Dummy = g_Config.m_ClDummy;
	const int FirstTick = Client()->GameTick(Dummy) + 1;
	const int PredTick = Client()->PredGameTick(Dummy);
	CNetObj_PlayerInput Last;
	mem_zero(&Last, sizeof(Last));
	if(const int *pInput = Client()->GetInput(FirstTick - 1, 0))
		mem_copy(&Last, pInput, sizeof(Last));
	for(int Tick = FirstTick; Tick < SearchTick; Tick++)
	{
		CNetObj_PlayerInput Input;
		const auto It = m_Committed.find(Tick);
		if(Tick >= PredTick && It != m_Committed.end())
		{
			Input = It->second;
		}
		else if(Tick >= PredTick)
		{
			Input = NeutralInput(Last);
			m_Committed[Tick] = Input;
		}
		else if(const int *pInput = Client()->GetInput(Tick, 0))
		{
			mem_copy(&Input, pInput, sizeof(Input));
		}
		else
		{
			Input = Last;
		}
		if(!SimulateTick(m_pRoot.get(), Tick, Input))
			return false;
		Last = Input;
	}

	m_RootInput = Last;
	m_SearchTick = SearchTick;
	const CCharacter *pChar = m_pRoot->GetCharacterById(m_LocalId);
	const CCharacterCore *pCore = pChar->Core();
	m_RootFrozen = pChar->m_FreezeTime > 0 || pCore->m_IsInFreeze || pCore->m_DeepFrozen || pCore->m_LiveFrozen;

	SNode Root;
	Root.m_pWorld = AcquireWorld();
	DetachedCopy(Root.m_pWorld.get(), m_pRoot.get());
	Root.m_LastInput = Last;
	Root.m_Cost = StateCost(pChar, 0);
	Root.m_Warm = !m_vWarmPlan.empty();
	m_vBeam.push_back(std::move(Root));
	m_Depth = 0;
	m_ParentIndex = 0;
	m_Searching = true;
	return true;
}

void CAutopilot::SampleActions(const SNode &Parent, const CCharacter *pChar, bool &HasWarm)
{
	m_vActions.clear();
	const CCharacterCore *pCore = pChar->Core();
	const bool Hooking = Parent.m_LastInput.m_Hook != 0 && pCore->m_HookState != HOOK_IDLE && pCore->m_HookState != HOOK_RETRACTED;
	const int KeepHook = Hooking ? HOOKMODE_HOLD : HOOKMODE_RELEASE;
	const float Guide = GuideAngle(pCore->m_Pos);
	const float GuideX = std::cos(Guide);
	const int GuideDir = GuideX > 0.2f ? 1 : (GuideX < -0.2f ? -1 : 0);

	// the rest of the previous best plan comes first
	HasWarm = Parent.m_Warm && m_Depth < (int)m_vWarmPlan.size();
	if(HasWarm)
		m_vActions.push_back(m_vWarmPlan[m_Depth]);

	// plain running and jumping
	for(int Dir = -1; Dir <= 1; Dir++)
	{
		for(int Jump = 0; Jump < 2; Jump++)
		{
			SAction Action;
			Action.m_Direction = Dir;
			Action.m_Jump = Jump != 0;
			Action.m_HookMode = KeepHook;
			Action.m_Angle = Guide;
			m_vActions.push_back(Action);
		}
	}

	// hooks along the route
	for(float Offset : {0.0f, -0.4f, 0.4f, -0.8f, 0.8f})
	{
		SAction Action;
		Action.m_Direction = GuideDir;
		Action.m_HookMode = HOOKMODE_NEW;
		Action.m_Angle = Guide + Offset;
		m_vActions.push_back(Action);
	}

	// weapons that can push the tee: grenade jumps, shotgun and laser bounces
	int aWeapons[3];
	int NumWeapons = 0;
	if(g_Config.m_ClAutopilotWeapons)
		for(int Weapon : {WEAPON_GRENADE, WEAPON_SHOTGUN, WEAPON_LASER})
			if(pCore->m_aWeapons[Weapon].m_Got && pCore->m_aWeapons[Weapon].m_Ammo != 0)
				aWeapons[NumWeapons++] = Weapon;

	// random exploration around the route
	while((int)m_vActions.size() < m_Samples)
	{
		SAction Action;
		Action.m_Direction = (int)(m_Rng() % 3) - 1;
		Action.m_Jump = RandomFloat() < 0.25f;
		const float HookRoll = RandomFloat();
		if(Hooking)
			Action.m_HookMode = HookRoll < 0.55f ? HOOKMODE_HOLD : (HookRoll < 0.8f ? HOOKMODE_RELEASE : HOOKMODE_NEW);
		else
			Action.m_HookMode = HookRoll < 0.5f ? HOOKMODE_RELEASE : HOOKMODE_NEW;
		Action.m_Angle = RandomFloat() < 0.6f ? Guide + (RandomFloat() - 0.5f) * 2.4f : RandomFloat() * 2.0f * pi;
		if(NumWeapons > 0 && RandomFloat() < 0.2f)
		{
			Action.m_Weapon = aWeapons[m_Rng() % NumWeapons];
			Action.m_HookMode = KeepHook;
			// to be pushed along the route, shoot against it most of the time
			if(RandomFloat() < 0.6f)
				Action.m_Angle = Guide + pi + (RandomFloat() - 0.5f) * 1.6f;
		}
		m_vActions.push_back(Action);
	}
}

void CAutopilot::ExpandNode(SNode &Parent)
{
	const CCharacter *pParentChar = Parent.m_pWorld->GetCharacterById(m_LocalId);
	if(!pParentChar)
		return;

	bool HasWarm = false;
	SampleActions(Parent, pParentChar, HasWarm);
	const int StartTick = m_SearchTick + Parent.m_Elapsed;
	for(size_t i = 0; i < m_vActions.size(); i++)
	{
		const SAction &Action = m_vActions[i];
		SNode Child;
		Child.m_pWorld = AcquireWorld();
		DetachedCopy(Child.m_pWorld.get(), Parent.m_pWorld.get());
		Child.m_LastInput = Parent.m_LastInput;
		const SSimResult Result = SimulateAction(Child.m_pWorld.get(), StartTick, Action, Child.m_LastInput, nullptr, nullptr);
		Child.m_Elapsed = Parent.m_Elapsed + Result.m_Ticks;
		Child.m_Warm = HasWarm && i == 0;
		const CCharacter *pChar = Child.m_pWorld->GetCharacterById(m_LocalId);

		if(Result.m_Outcome != OUTCOME_NONE)
		{
			const float Cost = TerminalCost(Result, pChar, Child.m_Elapsed);
			if(Cost < m_BestTerminalCost)
			{
				m_BestTerminalCost = Cost;
				m_BestTerminalOutcome = Result.m_Outcome;
				m_vBestTerminalPlan = Parent.m_vPlan;
				m_vBestTerminalPlan.push_back(Action);
			}
			ReleaseWorld(std::move(Child.m_pWorld));
			continue;
		}

		Child.m_Cost = StateCost(pChar, Child.m_Elapsed);
		Child.m_vPlan = Parent.m_vPlan;
		Child.m_vPlan.push_back(Action);
		m_vChildren.push_back(std::move(Child));
	}
}

void CAutopilot::SelectNextBeam()
{
	ReleaseNodes(m_vBeam);
	std::sort(m_vChildren.begin(), m_vChildren.end(), [](const SNode &a, const SNode &b) { return a.m_Cost < b.m_Cost; });

	std::unordered_set<uint64_t> Seen;
	// keep following the previous best plan while it is alive, it is the fallback
	for(SNode &Child : m_vChildren)
	{
		if(Child.m_Warm)
		{
			if(const CCharacter *pChar = Child.m_pWorld->GetCharacterById(m_LocalId))
				Seen.insert(StateKey(pChar));
			m_vBeam.push_back(std::move(Child));
			break;
		}
	}
	for(SNode &Child : m_vChildren)
	{
		if(!Child.m_pWorld)
			continue;
		if((int)m_vBeam.size() < m_BeamWidth)
		{
			// near-identical states waste the beam
			const CCharacter *pChar = Child.m_pWorld->GetCharacterById(m_LocalId);
			if(pChar && Seen.insert(StateKey(pChar)).second)
			{
				m_vBeam.push_back(std::move(Child));
				continue;
			}
		}
		ReleaseWorld(std::move(Child.m_pWorld));
	}
	m_vChildren.clear();
}

void CAutopilot::StepSearch(int64_t Deadline)
{
	while(m_Searching && m_Depth < m_MaxDepth)
	{
		if(m_ParentIndex < m_vBeam.size())
		{
			if(time_get() >= Deadline)
				return;
			ExpandNode(m_vBeam[m_ParentIndex]);
			m_ParentIndex++;
			continue;
		}
		SelectNextBeam();
		m_Depth++;
		m_ParentIndex = 0;
		if(m_vBeam.empty())
			return;
	}
}

void CAutopilot::FinalizeSearch()
{
	// best plan among the finished layer, the partially expanded one and the terminal states
	float BestCost = INF_DIST;
	const std::vector<SAction> *pBestPlan = nullptr;
	int Outcome = OUTCOME_NONE;
	for(const std::vector<SNode> *pNodes : {&m_vBeam, &m_vChildren})
	{
		for(const SNode &Node : *pNodes)
		{
			if(Node.m_pWorld && !Node.m_vPlan.empty() && Node.m_Cost < BestCost)
			{
				BestCost = Node.m_Cost;
				pBestPlan = &Node.m_vPlan;
			}
		}
	}
	if(!m_vBestTerminalPlan.empty() && m_BestTerminalCost < BestCost)
	{
		BestCost = m_BestTerminalCost;
		pBestPlan = &m_vBestTerminalPlan;
		Outcome = m_BestTerminalOutcome;
	}
	std::vector<SAction> vPlan;
	if(pBestPlan)
		vPlan = *pBestPlan;
	m_PlanOutcome = Outcome;

	// replay the plan to get the exact inputs and the path to draw
	std::unique_ptr<CGameWorld> pWorld = AcquireWorld();
	DetachedCopy(pWorld.get(), m_pRoot.get());
	CNetObj_PlayerInput Last = m_RootInput;
	CNetObj_PlayerInput LastCommitted = m_RootInput;
	m_vDebugPath.clear();
	if(const CCharacter *pChar = pWorld->GetCharacterById(m_LocalId))
		m_vDebugPath.push_back(pChar->Core()->m_Pos);
	m_ExpectedTick = -1;
	int Committed = 0;
	int Elapsed = 0;
	for(size_t i = 0; i < vPlan.size(); i++)
	{
		std::vector<CNetObj_PlayerInput> vInputs;
		const SSimResult Result = SimulateAction(pWorld.get(), m_SearchTick + Elapsed, vPlan[i], Last, i == 0 ? &vInputs : nullptr, &m_vDebugPath);
		if(i == 0)
		{
			for(const CNetObj_PlayerInput &Input : vInputs)
			{
				m_Committed[m_SearchTick + Committed++] = Input;
				LastCommitted = Input;
			}
			const CCharacter *pChar = pWorld->GetCharacterById(m_LocalId);
			if(Result.m_Outcome == OUTCOME_NONE && pChar)
			{
				m_ExpectedTick = m_SearchTick + Committed;
				m_ExpectedPos = pChar->Core()->m_Pos;
			}
		}
		Elapsed += Result.m_Ticks;
		if(Result.m_Outcome != OUTCOME_NONE)
			break;
	}
	ReleaseWorld(std::move(pWorld));

	// the plan ended early (or there was none): keep going without new presses
	while(Committed < m_Step)
	{
		LastCommitted = Committed == 0 ? NeutralInput(LastCommitted) : LastCommitted;
		LastCommitted.m_Jump = 0;
		m_Committed[m_SearchTick + Committed++] = LastCommitted;
	}

	m_CommitEnd = m_SearchTick + m_Step;
	m_vWarmPlan.assign(vPlan.size() > 1 ? vPlan.begin() + 1 : vPlan.end(), vPlan.end());
	ReleaseNodes(m_vBeam);
	ReleaseNodes(m_vChildren);
	m_Searching = false;
}

void CAutopilot::OnUpdate()
{
	if(!g_Config.m_ClAutopilot)
	{
		Stop("Autopilot disengaged");
		return;
	}
	if(Client()->State() != IClient::STATE_ONLINE)
	{
		Stop(nullptr);
		return;
	}

	const int LocalId = GameClient()->m_Snap.m_LocalClientId;
	if(LocalId < 0 || !GameClient()->m_Snap.m_aCharacters[LocalId].m_Active || !GameClient()->m_GameWorld.GetCharacterById(LocalId))
	{
		// dead or spectating, wait for the tee to come back
		ResetPlan();
		m_HaveLastActualPos = false;
		return;
	}
	if(!IsAllowed())
	{
		g_Config.m_ClAutopilot = 0;
		Stop(nullptr);
		GameClient()->Echo("Autopilot is only available on local/LAN servers or in /practice mode");
		return;
	}
	if(!m_Active)
		Start();
	if(LocalId != m_LocalId)
	{
		ResetPlan();
		m_LocalId = LocalId;
	}
	const int Dummy = g_Config.m_ClDummy;
	GameClient()->m_Controls.m_aMouseInputType[Dummy] = CControls::EMouseInputType::AUTOMATED;

	// done once the real tee crosses the finish line
	if(const CNetObj_Character *pLocal = GameClient()->m_Snap.m_pLocalCharacter)
	{
		const vec2 ActualPos(pLocal->m_X, pLocal->m_Y);
		if(m_HaveLastActualPos && distance(m_LastActualPos, ActualPos) < 32.0f * 20.0f)
		{
			const int Steps = std::clamp((int)std::ceil(distance(m_LastActualPos, ActualPos) / 12.0f), 1, 64);
			for(int s = 1; s <= Steps; s++)
			{
				if(TileFlagsAt(mix(m_LastActualPos, ActualPos, s / (float)Steps)) & TILEFLAG_FINISH)
				{
					g_Config.m_ClAutopilot = 0;
					Stop("Autopilot: finish reached");
					return;
				}
			}
		}
		m_LastActualPos = ActualPos;
		m_HaveLastActualPos = true;
	}

	const int PredTick = Client()->PredGameTick(Dummy);
	const int Step = g_Config.m_ClAutopilotStep;
	const int64_t Deadline = time_get() + time_freq() * g_Config.m_ClAutopilotBudget / 1000;
	for(int Round = 0; Round < 8; Round++)
	{
		if(!m_Searching)
		{
			// enough is decided already
			if(m_CommitEnd > PredTick + 3 * Step)
				break;
			if(!StartSearch(std::max(m_CommitEnd, PredTick + 2)))
				break;

			// the real tee is not where the committed inputs expected it (server correction,
			// teleporter, other players): throw away what is not sent yet and plan from here
			const CCharacter *pRootChar = m_pRoot->GetCharacterById(m_LocalId);
			if(m_ExpectedTick == m_SearchTick && pRootChar && distance(pRootChar->Core()->m_Pos, m_ExpectedPos) > DESYNC_DISTANCE)
			{
				m_Committed.erase(m_Committed.upper_bound(PredTick), m_Committed.end());
				m_vWarmPlan.clear();
				m_CommitEnd = -1;
				m_ExpectedTick = -1;
				if(!StartSearch(PredTick + 2))
					break;
			}
		}
		StepSearch(Deadline);
		const bool Done = m_Depth >= m_MaxDepth || m_vBeam.empty();
		if(Done || PredTick + 1 >= m_SearchTick)
			FinalizeSearch();
		if(time_get() >= Deadline)
			break;
	}
	// the next tick needs this decision now, even when the time budget is used up
	if(m_Searching && PredTick + 1 >= m_SearchTick)
		FinalizeSearch();
}

void CAutopilot::OverrideInput(int Tick, CNetObj_PlayerInput *pInput)
{
	CNetObj_PlayerInput Input;
	const auto It = m_Committed.find(Tick);
	if(It != m_Committed.end())
		Input = It->second;
	else
		Input = NeutralInput(*pInput);

	pInput->m_Direction = Input.m_Direction;
	pInput->m_TargetX = Input.m_TargetX;
	pInput->m_TargetY = Input.m_TargetY;
	pInput->m_Jump = Input.m_Jump;
	pInput->m_Fire = Input.m_Fire;
	pInput->m_Hook = Input.m_Hook;
	pInput->m_WantedWeapon = Input.m_WantedWeapon;

	// let the cursor show where the autopilot aims
	GameClient()->m_Controls.m_aMousePos[g_Config.m_ClDummy] = vec2(Input.m_TargetX, Input.m_TargetY);

	m_Committed.erase(m_Committed.begin(), m_Committed.lower_bound(Tick - 2 * SERVER_TICK_SPEED));
}

void CAutopilot::OnRender()
{
	if(!m_Active || Client()->State() != IClient::STATE_ONLINE)
		return;

	if(g_Config.m_ClAutopilotDebug && m_vDebugPath.size() >= 2)
	{
		std::vector<IGraphics::CLineItem> vLines;
		vLines.reserve(m_vDebugPath.size() - 1);
		for(size_t i = 1; i < m_vDebugPath.size(); i++)
			vLines.emplace_back(m_vDebugPath[i - 1], m_vDebugPath[i]);

		Graphics()->TextureClear();
		Graphics()->LinesBegin();
		if(m_PlanOutcome == OUTCOME_FINISH)
			Graphics()->SetColor(1.0f, 0.85f, 0.2f, 0.9f);
		else if(m_PlanOutcome == OUTCOME_FROZEN || m_PlanOutcome == OUTCOME_DEAD)
			Graphics()->SetColor(1.0f, 0.3f, 0.3f, 0.9f);
		else
			Graphics()->SetColor(0.3f, 1.0f, 0.5f, 0.9f);
		Graphics()->LinesDraw(vLines.data(), vLines.size());
		Graphics()->LinesEnd();
	}

	const char *pStatus = "planning";
	if(m_PlanOutcome == OUTCOME_FINISH)
		pStatus = "finish in sight";
	else if(m_PlanOutcome == OUTCOME_FROZEN || m_PlanOutcome == OUTCOME_DEAD)
		pStatus = "no safe move found";
	else if(m_PlanOutcome == OUTCOME_TELEPORT)
		pStatus = "heading into teleporter";
	char aBuf[128];
	str_format(aBuf, sizeof(aBuf), "AUTOPILOT - %s", pStatus);

	const CScreenRect WorldScreen = Graphics()->GetScreen();
	Ui()->MapScreen();
	const float FontSize = 8.0f;
	const float Width = TextRender()->TextWidth(FontSize, aBuf);
	TextRender()->TextColor(0.4f, 1.0f, 0.6f, 1.0f);
	TextRender()->Text(Ui()->Screen()->w / 2.0f - Width / 2.0f, Ui()->Screen()->h - 40.0f, FontSize, aBuf);
	TextRender()->TextColor(TextRender()->DefaultTextColor());
	Graphics()->MapScreen(WorldScreen);
}

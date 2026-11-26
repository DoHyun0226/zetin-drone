# 노션 드론 페이지 재정리 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 노션 `드론` 페이지와 하위 전부를 설계안의 파트별 구조·제목·아이콘·본문 규칙으로 정렬하고, 그 규칙을 `docs/notion_doc_convention.md` 에 쓴다.

**Architecture:** 먼저 모든 페이지를 저장하고 페이지 ID 대조표를 만든다(Task 1). 노션 변경은 대조표를 정답으로 삼아 진행하고, 끝에 같은 대조표로 검사 스크립트가 결과를 판정한다(Task 7). 레포 변경은 컨벤션 문서와 지침 포인터뿐이다.

**Tech Stack:** Notion MCP(`notion-fetch`, `notion-update-page`, `notion-create-pages`, `notion-move-pages`, `notion-create-file-upload`, `notion-get-file-download-urls`, `notion-get-users`, `notion-delete-pages`), curl, `python -I`.

**Spec:** `docs/superpowers/specs/2026-11-13-notion-structure-design.md`

## Global Constraints

- 노션은 ZETIN 워크스페이스 `a622d27b-08d3-81e5-bf6c-0003781fc028` 만. 각 작업 묶음 시작 때 `get-self` 로 확인, 다르면 중단.
- 드론 페이지 `3142d27b08d3804eb85ced1ca13d8998` 의 골격(`활동 한 줄 소개` → 현재 상태 → `## 📋 결과` → `## ⚠️ 안전 수칙` → `## 🗓 활동 로그 · 마일스톤`)은 그대로.
- 기록형 4쪽은 본문 불변(머리 콜아웃·백로그 절 포함). 허용: 제목·아이콘·위치, 깨진 글자, 끊긴 링크.
- 아이콘: 파트 토글 🔧 HW · 💻 SW · 📐 CAD · 🛩️ 공통, 페이지 📄 설명형 · 📅 기록형 · 📎 자료 · ☑️ 할 일.
- **커밋하지 않는다**(사용자 지시 2026-11-13). 레포 파일은 고치기만 하고 커밋·push 없음.
- 레포는 공개다. 컨벤션·계획 문서에 팀원 이름·학번을 쓰지 않는다.
- 작업 파일은 전부 `$W` = 세션 스크래치패드의 `notion-restructure/` 폴더(레포 밖) 아래. 업로드 토큰은 파일에 쓰지 않는다.
- 업로드 한도 20 MiB(MCP 단일 업로드). 6 MiB 는 2026-11-13 성공 확인.

## Review Focus

1. **`<page>` 가 `<mention-page>` 로 조용히 바뀜** — 토글 안 배치 때 대상이 그 부모의 실제 자식이 아니면 이동 없이 링크만 생긴다. Task 7 검사 (a)가 `<page>` 와 토글 경로를 같이 본다.
2. **본문 정리 중 측정값·조건 문장 유실** — 머리 콜아웃 안의 측정값·주의 문구. Task 5 는 콜아웃을 지우기 전에 옮길 곳을 정하고, Task 7 diff 검사가 허용 목록 밖의 삭제를 실패로 낸다.
3. **업로드는 됐는데 블록에 안 붙어 만료** — 붙이지 않은 업로드는 1시간 뒤 사라진다. Task 4 는 업로드 직후 같은 묶음 안에서 붙이고, Task 7 이 파일 블록 수를 센다.
4. **같은 내용의 파일 두 번 올림** — Task 3 의 SHA-256 대조 결과대로만 올린다.
5. **드론 페이지 통째 교체 시 기존 파일 블록·mention 유실** — 드론 페이지는 `update_content` 의 부분 치환만 쓰고(`replace_content` 금지), 거버 zip 파일 블록은 `notion-file-block://` 원문을 그대로 옮긴다.

---

### Task 1: 사전 확인 · 저장 · 대조표

**Files:**
- Create: `$W/before/<page_id>.md`(페이지마다), `$W/mapping.tsv`, `$W/attachments/` + `$W/attachments.tsv`, `$W/sharing.md`
- Create: `$W/tools/check_mapping.py`

**Interfaces:**
- Produces: `mapping.tsv` 열 = `page_id	old_parent	old_title	new_parent	toggle_path	new_title	icon	kind`(kind ∈ 설명형·기록형·자료·할일·삭제). `toggle_path` 는 드론 페이지 기준 `결과 › 💻 SW › 펌웨어` 꼴, 하위 페이지면 빈 값. 새로 만들 페이지는 `page_id` 에 `NEW:<slug>`.
- Produces: `attachments.tsv` 열 = `page_id	block_ref(notion-file-block://… 원문)	filename	sha256	bytes`.

- [ ] **Step 1:** `get-self` → workspace id 가 위 값인지 확인.
- [ ] **Step 2:** 드론 페이지부터 `<page url>` 을 따라 하위 전부를 재귀 fetch 해 `before/<id>.md` 로 저장(fetch 결과의 `text` 그대로). 기대: 19쪽 + 그 아래에서 새로 나온 것.
- [ ] **Step 3:** `mapping.tsv` 작성. 설계안 5절 트리의 모든 줄 + Step 2 에서 나온 모든 페이지. 2026-10-10 빈 회의 노트(`3f52d27b08d3805ea561c8ffa9952708`)는 kind=삭제.
- [ ] **Step 4:** `tools/check_mapping.py <before_dir> <mapping.tsv>` — before 의 모든 페이지 ID 가 mapping 에 정확히 한 번, NEW 가 아닌 mapping ID 가 모두 before 에 있으면 exit 0. 실행해 exit 0 확인.
- [ ] **Step 5:** 첨부가 있는 페이지마다 `notion-get-file-download-urls` 로 받아 `attachments/<page_id>/` 에 저장, `attachments.tsv` 작성.
- [ ] **Step 6:** `notion-get-users` 로 워크스페이스 게스트 확인. 업로드 대상 페이지(드론, 「2026-2_drone_ver.1」, 「CAD file」, 「Drone Legacy Modeling」)마다 웹 공개·초대 사용자·게스트·하위 페이지 별도 공유를 확인한다. MCP 로 안 보이는 항목은 페이지 목록을 붙여 사용자에게 한 번에 묻는다(공유 메뉴 확인 요청). 결과를 `sharing.md` 에. 팀 밖 공유가 있는 페이지는 업로드하지 않고 멈춘다.

### Task 2: 컨벤션 문서 개정

**Files:**
- Modify: `docs/notion_doc_convention.md`(전면 개정)
- Modify: `.claude/CLAUDE.md`, `AGENTS.md`(포인터 마커 블록)

**Interfaces:**
- Consumes: 설계안 4·5·6절.

- [ ] **Step 1:** 컨벤션 문서를 설계안 6절 규칙으로 다시 쓴다. 절 구성: 0 레포가 정답(옛 §0 유지) / 1 구조(파트 토글·대상 페이지, 트리, MCP 배치 절차를 "실제 부모로 move → 토글 안 `<page>`" 로 일반화) / 2 페이지 유형(설명형·기록형·자료·할 일; 기록형 불변 범위 명시) / 3 제목 / 4 아이콘 / 5 본문(머리 콜아웃 없음, 설명형 `레포:`·`확인:` 줄, 분류 토글 규칙, 한 페이지 한 곳) / 6 표기(옛 §4, 콜아웃 아이콘 목록에서 🚦 삭제) / 7 갱신 시점(옛 §5 개정) / 8 안전 서술(옛 §6) / 9 드론 페이지 맨 위(옛 §8 개정: 현재 상태 네 줄 각 한 줄, 링크 줄·📌 없음) / 10 이 문서가 원본(옛 §7).
- [ ] **Step 2:** 두 지침 파일의 마커 블록 핵심 요약을 새 규칙으로 바꾼다(파트 토글 HW·SW·CAD·공통, 제목 짧게·기록형 날짜 앞, 머리 콜아웃 없음). 두 블록 글자가 같아야 한다.
- [ ] **Step 3:** 확인 — 두 파일에서 마커 사이를 뽑아 `diff` 가 비어 있음. `grep -n "🚦\|CODE 안의" docs/notion_doc_convention.md .claude/CLAUDE.md AGENTS.md` 결과 없음.

### Task 3: 올릴 파일 준비 · 중복 판정

**Files:**
- Create: `$W/upload/` 아래 파일들, `$W/uploads.tsv`

**Interfaces:**
- Consumes: `attachments.tsv`(Task 1).
- Produces: `uploads.tsv` 열 = `target(mapping new_title)	local_path(실제로 올릴 산출물 경로)	source_paths(원본 경로들, `;` 구분)	sha256	bytes	decision`(decision ∈ upload·skip-same·exclude), **올릴 산출물 하나에 한 줄**.
- Produces: `required_files.txt` — 설계안 7절 표와 "올리지 않는 것"에 해당하는 로컬 파일 전부(`docs/materials` 기준 경로, 디렉터리는 `find -type f` 로 펼침: 드론시각화 mp4 9개, 10/30 성과공유 PPT·PDF, STL 18개, `.gcode.3mf` 4개, 레거시 zip 내 파일 등).

- [ ] **Step 1:** 만들기: `FINAL.step.zip`(zip -9), STL 18개를 `drone_v1_frame_stl.zip` 하나로, `Drone-Dev-Kit_V4 BOM.csv` 를 UTF-8 로(`iconv -f UTF-16 -t UTF-8`, 이름 `Drone-Dev-Kit_V4_BOM.csv`).
- [ ] **Step 2:** 대조: `drone_v1_frame.step` 과 노션 「CAD file」 의 같은 이름 파일, STL 들과 `stl_print_parts.zip` 안의 STL, 레거시 drive zip 의 각 파일과 「Drone Legacy Modeling」 첨부(또는 그 안의 zip 내용)를 SHA-256 으로 비교.
- [ ] **Step 3:** `required_files.txt` 와 `uploads.tsv` 작성 — required 의 모든 줄이 uploads 의 `source_paths` 에 정확히 한 번 나온다(zip 으로 묶거나 UTF-8 로 바꾼 것은 원본이 source_paths, 산출물이 local_path). 7절 "올리지 않는 것"은 exclude 로 적는다. 20 MiB 넘는 upload 줄이 없어야 한다(`awk -F '\t' '$6=="upload" && $5>20971520' "$W/uploads.tsv"` 결과 없음).

### Task 4: 새 페이지 만들기 · 파일 올리기

**Interfaces:**
- Consumes: `mapping.tsv` 의 `NEW:` 줄, `uploads.tsv`.
- Produces: `mapping.tsv` 의 `NEW:<slug>` 를 실제 page_id 로 바꾼 값.

- [ ] **Step 1:** 드론 페이지를 부모로 `📎 PCB v1.5.2`, `📎 Drone-Dev-Kit V4`, `📄 시작하기`, `☑️ 할 일`, `📎 발표·보고서·영상` 을 만들고, 「2026-2_drone_ver.1」 을 부모로 `📎 최종 형상 (2026-11)` 을 만든다. 만든 즉시 mapping 갱신.
- [ ] **Step 2:** 파일마다 `notion-create-file-upload` → curl POST(헤더는 명령줄에만) → 같은 묶음에서 대상 페이지에 `<file source="file-upload://…" />` 와 설명 한 줄(무엇·어느 버전, 모르면 "대응 미확인")을 붙인다. 이미지(`pcb_*.png`, 렌더 jpg)는 `<image>`. 결과를 `uploads.tsv` 에 `done`/`failed` 로.
- [ ] **Step 2b:** 거버 zip 옮기기: 드론 페이지 PCB 토글의 `<file source="notion-file-block://3e22d27b-08d3-80fc-…/fc87b154-…&name=1.5.2_drone%28modify%29.zip" />` 원문을 PCB v1.5.2 페이지에 그대로 넣는다. PCB 페이지를 다시 fetch 해 그 블록을 `get-file-download-urls` 로 받아 `attachments.tsv` 의 해시와 같은지 확인한다. 같지 않으면 로컬 `docs/materials/hw/pcb-v1.5.2/gerber_1.5.2_drone(modify).zip` 을 새로 올린다(해시가 노션 원본과 같은지 먼저 대조). 원래 블록은 Task 6 에서만 지운다.
- [ ] **Step 3:** `☑️ 할 일`: 출처 = `docs/materials/plans/2026-11-12_future_plan.md`, 지상국 페이지의 현재 할 일, 기록형 백로그의 열린 항목. HW·SW·CAD·공통으로 묶은 체크박스, 기록에서 온 항목엔 출처 mention. 담당자·일정은 확인된 것만.
- [ ] **Step 4:** `📄 시작하기`: 지상국 페이지의 처음 참여·작업 순서 절을 옮긴다(옮긴 문장 목록을 `$W/changes.md` 에 기록).
- [ ] **Step 5:** `📎 발표·보고서·영상`: 발표 사이트·저장소 링크 + 업로드 파일.

### Task 5: 옮기기 · 제목 · 아이콘 · 본문 정리

**Interfaces:**
- Consumes: `mapping.tsv`.
- Produces: `$W/changes.md` — 페이지별 허용 변경 목록(종류: 메타 삭제 / 콜아웃→확인 줄 / 측정값 이동(옮긴 위치) / 오타 / 링크 / 절 이동), Task 7 이 읽는다. 줄 형식 `page_id	kind	before_text	after_location`.

- [ ] **Step 1:** 「CAD file」 을 `ver.1 (2026-09)` 로 이름·아이콘 📎. 다른 페이지는 부모가 그대로라 move 없음 — mapping 에서 old_parent≠new_parent 인 줄만 `notion-move-pages`.
- [ ] **Step 2:** mapping 대로 모든 페이지 제목·아이콘을 `update-page` 로.
- [ ] **Step 3:** 설명형마다: 머리 콜아웃의 상태·검증 날짜·레포 경로 → 맨 아래 `레포: … · 확인: …` 한 줄, 측정값·주의 문구는 본문 해당 절로, 메타 문장 삭제. 「기체 구성」의 "다음 실물 점검에 남길 것"·예비 비용 토글은 유지, "PCB·CAD 원본 링크가 없으므로…" 처럼 이제 틀린 문장은 고친다. 10-30 기록의 없는 `flow_hold` 경로 링크 → `firmware/flight/zetin_fc`. 각 변경을 `changes.md` 에.
- [ ] **Step 3b:** 기존 자료 페이지(「2026-2 기체」, 「ver.1 (2026-09)」, 「2026-1 기체」)의 파일 블록마다 바로 아래에 무엇·어느 버전 한 줄을 단다(모르면 "대응 미확인"). 메타 문장("위치 확인이 필요하다" 등)은 지운다. `changes.md` 에 기록.
- [ ] **Step 4:** 기록형 4쪽: 08-01 기록 콜아웃의 깨진 글자만 고친다("처회된 관상" 등 — 원래 말이 문맥상 분명할 때만, 아니면 `[판독 불가]` 표시 없이 그대로 두고 보고). 다른 diff 없음.
- [ ] **Step 5:** 빈 회의 노트 `3f52d27b08d3805ea561c8ffa9952708` 를 `notion-delete-pages` 로 휴지통에. 「기체 구성」 본문의 그 `<page>` 줄도 사라졌는지 확인.

### Task 6: 드론 페이지 맨 위 정리

**Interfaces:**
- Consumes: Task 4 의 새 page_id, mapping 의 toggle_path.

- [ ] **Step 1:** 모든 대상 페이지가 드론 페이지(또는 mapping 의 new_parent)의 실제 자식인지 fetch 로 확인.
- [ ] **Step 2:** `## 📋 결과` 부터 `## ⚠️ 안전 수칙` 직전까지를 `update_content` 한 번으로 바꾼다: 파트 토글 네 개, 하위 분류 토글(펌웨어·지상국·비행 기록), mapping 순서대로 `<page url>`, SW 토글 첫 줄 레포 링크 하나. 거버 파일 블록은 PCB v1.5.2 페이지로 옮겼으니 여기서 뺀다(Task 4 에서 옮긴 것을 확인한 뒤에만).
- [ ] **Step 3:** 현재 상태 콜아웃을 네 줄(각 한 줄)로 줄이고 링크는 mention 으로. 활동 로그의 옛 제목 mention 은 자동 갱신되므로 그대로.

### Task 7: 검사 · 보고

**Files:**
- Create: `$W/after/<page_id>.md`, `$W/tools/verify_notion.py`, `$W/report.md`

**Interfaces:**
- Consumes: `before/`, `after/`, `after_files/`, `mapping.tsv`, `changes.md`, `attachments.tsv`, `uploads.tsv`, `required_files.txt`.

- [ ] **Step 1:** 드론 페이지부터 재귀 fetch → `after/`.
- [ ] **Step 2:** `verify_notion.py <W>` (W 아래 고정 이름 파일들을 읽음) 를 작성·실행. 검사:
  - (a) mapping 의 각 페이지가 new_parent 의 `<page>`(mention 아님)이고, toggle_path 의 토글 안에 있다. 삭제 줄은 어디에도 없다.
  - (b) 제목·아이콘이 mapping 과 같다. 설명형 첫 블록이 🚦·📌 콜아웃이 아니다.
  - (c) 페이지별 before↔after 줄 diff 의 삭제·변경 줄이 모두 `changes.md` 에 있다. 종류가 이동(측정값 이동·절 이동)인 줄은 `before_text`(공백 정규화)가 `after_location` 페이지의 after 본문에 실제로 들어 있어야 한다. 기록형은 제목·깨진 글자·링크 외 diff 없음.
  - (d) `attachments.tsv` 와 `uploads.tsv` 의 done 줄마다 after 의 최종 파일 블록을 `get-file-download-urls` 로 다시 받아 크기·SHA-256 이 같다(Step 1 직후 별도 단계로 받아 `$W/after_files/` 에 저장, 스크립트는 그 폴더를 읽는다). `required_files.txt` 의 모든 줄이 uploads 의 source_paths 에 정확히 한 번 있고, 그 줄의 decision 이 exclude 가 아니면 done 이나 skip-same 이다.
  - (e) 내부 링크: after 안의 노션 링크 ID 가운데 드론 하위를 가리키는 것은 after 페이지 집합에 있다(드론 밖 노션 링크는 fetch 가 성공하면 통과). 레포 링크 `github.com/lightminn/zetin-drone/blob/main/<path>` 는 `git cat-file -e origin/main:<path>`, 그 밖의 외부 링크는 `curl -sL -o /dev/null -w '%{http_code}'` 의 종료 코드가 0 이고 200 ≤ HTTP 코드 < 400 일 때만 통과(`000`·연결 실패는 실패).
  - (f) 📎 자료 페이지의 파일·이미지 블록마다 바로 다음 블록이 설명 한 줄이다.
  모든 검사 통과 시 exit 0, 아니면 실패 항목을 찍는다.
- [ ] **Step 3:** 실패 항목을 고치고 Step 1~2 반복, exit 0 까지.
- [ ] **Step 4:** `report.md` 와 사용자 보고: 옮긴 페이지 수, 올린·생략·제외 파일, 고친 문장, 남은 것(V4 BOM 대응 미확인, 판독 못 한 글자 등). 레포 변경(컨벤션·지침·설계·계획)은 커밋하지 않았음을 밝힌다.

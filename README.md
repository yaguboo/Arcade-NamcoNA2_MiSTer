# Namco NA-1 / NA-2 — MiSTer FPGA 코어

> [!IMPORTANT]
> **취미로 만든 프로젝트입니다.**
> 개인이 순전히 취미로 만든 코어입니다. 버그 제보는 반갑게 받지만, **대응할 수도 있고 못 할 수도 있습니다.**
> 업데이트나 지원을 약속하지 않습니다.
>
> **이 코드로 이어서 개발하실 때는 꼭 출처를 남겨 주세요.**
> 이 저장소를 바탕으로 수정하거나 다른 코어를 만드실 때 출처(이 저장소 링크)를 밝혀 주시면 정말 감사하겠습니다.
> 라이선스(GPL-3.0)에 따라 원래의 저작권 표시와 이 프로젝트가 감사를 전한 분들의 출처도 함께 유지해 주세요.

1992~1994년 남코가 퍼즐·퀴즈·스포츠 게임에 쓰던 아케이드 기판 **Namco NA-1 / NA-2** 를
MiSTer (DE10-Nano) 에서 돌리는 FPGA 코어입니다. 소프트웨어 에뮬레이션 없이, 기판의
각 칩을 RTL 로 다시 만들었습니다.

| 블록 | 원래 기판 | 이 코어의 구현 |
|---|---|---|
| 메인 CPU | Motorola 68000 | fx68k (사이클 정확 68000 코어) |
| MCU | Namco C69 (NA-1) / C70 (NA-2) — Mitsubishi M37702 기반 | M37702 CPU 코어 + 직접 작성한 온칩 주변장치 (타이머 8개, 인터럽트 컨트롤러, I/O 포트, A-D). MCU 의 내장 BIOS 를 그대로 실행합니다 |
| 사운드 | Namco C219 (C140 계열) — 16 voice PCM, 샘플은 메인 work RAM 에서 읽음 | `na2_c219` |
| 비디오 | 블리터, 타일맵 4장 (8bpp/4bpp), 스프라이트, ROZ (회전·확대) 레이어, 픽셀 라인 모드, 5:5:5 팔레트 | `rtl/video/` |
| 보호 칩 | KEYCUS (C348~C394) — 게임 식별 + 난수원 | `na2_keycus`. Tinkle Pit 의 C367 은 알고리즘까지 구현 |
| NVRAM | 28C16 EEPROM (2 KB) | `na2_eeprom` + MiSTer NVRAM 저장 |

NA-1 과 NA-2 는 같은 기판이고 MCU 의 내장 마스크 ROM(BIOS) 만 다릅니다. 그래서
**비트스트림(RBF) 하나가 21개 세트를 모두 돌립니다.** 게임마다 다른 KEYCUS 식별값과
MCU BIOS 는 `.mra` 가 ROM 과 함께 내려 줍니다.

---

## 지원 게임

v0.9.2는 CRT 수직 위치 조절을 추가합니다. 이번 빌드로 21개 세트 모두 MiSTer
실기에서 부팅·타이틀·어트랙트 화면을 다시 확인했습니다. Numan Athletics(NA2),
Tinkle Pit(NA1)의 정지·재개 및 사용자 CRT 테스트도 통과했습니다.
아래 표의 "MAME 픽셀 일치" 는 이전 검증에서 실기 스크린샷이 MAME 0.289 가 그린 같은 장면과 68,096 픽셀 중
0 픽셀 차이였다는 뜻입니다 (세트당 최소 한 프레임).

| MAME set | 공식 명칭 | MCU | 상태 |
|---|---|---|---|
| `bkrtmaq` | Bakuretsu Quiz Ma-Q Dai Bouken (Japan) | C69 | 실기 동작, MAME 픽셀 일치 |
| `cgangpzl` | Cosmo Gang the Puzzle (US) | C69 | 실기 동작, MAME 픽셀 일치 |
| `cgangpzlj` | Cosmo Gang the Puzzle (Japan) | C69 | 실기 동작, MAME 픽셀 일치 |
| `exvania` | Exvania (World) | C69 | 실기 동작, MAME 픽셀 일치 |
| `exvaniaj` | Exvania (Japan) | C69 | 실기 동작, MAME 픽셀 일치 |
| `fghtatck` | Fighter & Attacker (US) | C69 | 실기 동작, MAME 픽셀 일치 |
| `fa` | F/A (Japan) | C69 | 실기 동작, MAME 픽셀 일치 |
| `swcourt` | Super World Court (World) | C69 | 실기 동작, MAME 픽셀 일치 |
| `swcourtj` | Super World Court (Japan) | C69 | 실기 동작, MAME 픽셀 일치 |
| `swcourtb` | Super World Court (World, bootleg) | C69 | 실기 동작, MAME 픽셀 일치 |
| `emeraldaj` | Emeraldia (Japan) | C69 | 실기 동작, MAME 픽셀 일치 |
| `emeraldajb` | Emeraldia (Japan Version B) | C69 | 실기 동작, MAME 픽셀 일치 |
| `tinklpit` | Tinkle Pit (Japan) | C69 | 실기 동작, MAME 픽셀 일치, 게임 플레이 확인 |
| `numanath` | Numan Athletics (World) | C70 | 실기 동작, MAME 픽셀 일치, 게임 플레이 확인 |
| `numanathj` | Numan Athletics (Japan) | C70 | 실기 동작, MAME 픽셀 일치, 게임 플레이 확인 |
| `knckhead` | Knuckle Heads (World) | C70 | 실기 동작, 게임 플레이 확인 (비교 프레임이 페이드 중이라 픽셀 일치는 판정 보류 — 그림은 같고 팔레트 단계만 다름) |
| `knckheadj` | Knuckle Heads (Japan) | C70 | 실기 동작, MAME 픽셀 일치 |
| `knckheadjp` | Knuckle Heads (Japan, Prototype?) | C70 | 실기 동작, MAME 픽셀 일치 |
| `emeralda` | Emeraldia (World) | C70 | 실기 동작, MAME 픽셀 일치 |
| `emeraldajd` | Emeraldia (Japan Version D) | C70 | 실기 동작, MAME 픽셀 일치 |
| `quiztou` | Nettou! Gekitou! Quiztou!! (Japan) | C70 | 실기 동작, MAME 픽셀 일치 |

사운드는 21개 세트 전부 실기에서 voice 가 MAME 와 같은 패턴으로 발음되는 것을 계측했고,
`numanath` 는 사람이 직접 BGM·효과음을 들어 확인했습니다. 다만 **샘플 단위 파형 대조는
아직 하지 않았습니다.** "게임 플레이 확인" 이 없는 세트는 어트랙트까지만 검증했다는
뜻이지 플레이가 안 된다는 뜻은 아닙니다.

**지원하지 않는 세트:** `xday2` 는 같은 계열이지만 기계 구성이 다릅니다 — 자체 메모리 맵,
8 KB EEPROM(28C64), MSM6242 실시간 시계가 추가돼 있어 이 코어에 없는 하드웨어가 필요합니다.
`zelos` / `zelost` / `bbbingo` / `bbbingot` 은 별도 구성(KEYCUS 없음)이며 MAME 에서도
`MACHINE_NOT_WORKING` 입니다.

## 설치 — ROM 만 넣으면 됩니다

소스 저장소의 `SD/` 폴더는 MiSTer SD 카드의 루트(`/media/fat/`)와 같은 구조입니다.
**릴리스의 `NamcoNA2_v0.9.2_SD.zip`을 풀고 `_Arcade/`와 `games/`를 SD 카드 루트에
그대로 복사**하면 됩니다. 소스 저장소를 받았다면 `SD/` 안의 내용을 복사합니다.

```
SD/_Arcade/cores/NamcoNA2.rbf                 코어 (빌드된 비트스트림)
SD/_Arcade/_Kaze's Cores/<게임 이름>.mra    게임 목록 (공식 명칭)
SD/games/mame/필요한_ROM.txt               넣어야 할 ROM zip 목록
```

1. `SD/` 의 내용을 SD 카드 루트에 복사합니다. 기존 파일은 덮어써도 됩니다.
2. 직접 마련한 MAME ROM 세트 zip 을 SD 카드의 `/games/mame/` 에 넣습니다.
   어떤 zip 이 필요한지는 `필요한_ROM.txt` 에 게임별로 적혀 있습니다.
   zip 은 MAME 세트 이름 그대로 두고, 압축을 풀지 않습니다.
3. MiSTer 메뉴에서 **Arcade → `_Kaze's Cores`** 로 들어가 게임을 고릅니다.

- `.rbf` 를 직접 실행하지 말고 `.mra` 로 실행하세요. ROM 로드와 DIP·OSD 기본값이
  `.mra` 에 들어 있습니다.
- 폴더 이름이 `_` 로 시작해야 MiSTer 메뉴에 보입니다. 이름을 바꾸지 마세요.

## CRT 화면 위치 조절

VGA/아날로그 CRT에서 화면 전체가 아래로 치우치면 OSD의 **CRT V-position**을
조절하세요. 기본값 **Original**은 기존 위치입니다. **Up 2 / 4 / 8 / 12 / 16 / 20 / 24**로
화면을 위로 옮길 수 있습니다. `Up 16`부터 비교하고 화면에 맞게 조절한 뒤
`Save settings`로 저장하세요. `Original`을 선택하면 즉시 기존 위치로 돌아갑니다.

이 옵션은 스캔더블러/HQ2x, 화면 회전 또는 Flip 사용 중에는 숨겨지고 보정도
해제됩니다. 일반 HDMI도 같은 입력 동기 신호를 사용하므로 선택값의 영향을
받을 수 있습니다. 값 변경 직후에는 화면 동기가 잠깐 안정되는 시간이 필요할 수 있습니다.


## 빌드 방법

필요한 것: **Intel Quartus Prime Lite Edition 17.0** (MiSTer 코어 표준 버전). 다른 버전에서도
합성은 될 수 있지만 검증한 버전은 17.0 입니다.

```sh
cd projects/namco/na1_na2/targets/mister
quartus_sh --flow compile NamcoNA2
```

결과물은 `projects/namco/na1_na2/targets/mister/output_files/NamcoNA2.rbf` 입니다. Quartus GUI 로
`NamcoNA2.qpf` 를 열고 Compile 을 눌러도 같습니다.

- 디렉터리 구조를 그대로 유지해야 합니다. 프로젝트 파일이 `../../../../../third_party`,
  `../../../../../platforms/mister/sys` 를 상대 경로로 찾습니다.
- `build_id.v` 는 빌드 시작 때 `platforms/mister/sys/build_id.tcl` 이 자동으로 만듭니다.
- Quartus 17.0 의 fitter 가 드물게 내부 오류로 죽으면서도 정상 종료 코드를 남기는 경우가 있습니다.
  `.rbf` 의 생성 시각과 로그 끝부분을 확인하고, 그런 경우 한 번 더 빌드하면 됩니다.

직접 빌드한 `.rbf` 는 `SD/_Arcade/cores/` 의 같은 이름 파일과 바꿔 넣으면 됩니다.

## 디렉터리 구성

원래 저장소의 상대 경로를 그대로 유지했습니다. 빌드에 실제로 쓰이는 파일만 들어 있습니다.

```
LICENSE                              GPL-3.0 전문
README.md                            이 문서
SD/                                  SD 카드 루트에 복사할 설치 파일 (RBF, MRA, ROM 목록)
projects/namco/na1_na2/
  rtl/                               기판 하드웨어 RTL (68000 버스, C69/C70 MCU, 비디오·사운드, 메모리 RTL)
  integration/                       ROM 다운로드 경로 (플랫폼 중립 어댑터)
  targets/mister/                    MiSTer 최상위 (.qpf .qsf .sdc .sv files.qip, PLL)
third_party/                         외부 IP (아래 "감사의 말과 사용한 코드" 참조)
platforms/mister/sys/                MiSTer framework (Template_MiSTer)
```

소스 주석에는 개발 중에 쓴 내부 문서 번호와 측정 기록이 그대로 남아 있습니다. 해당 개발
문서와 측정·분석 도구는 이 배포본에 포함하지 않았습니다. 빌드에는 영향이 없습니다.

## 작업 내역

### 2026-10-11 — 선택형 CRT 수직 위치 조절

- OSD에 `CRT V-position`을 추가했습니다. 기본값 `Original`을 유지하고, 필요한 CRT에서만 출력 수직 동기를 2~24라인 조절합니다.
- 게임 좌표, 262라인 프레임 주기, 게임 IRQ와 오디오 클럭을 유지합니다. 스캔더블러·회전·Flip 사용 시 보정을 해제합니다.
- 8개 옵션의 CRTC 및 실제 MiSTer 영상 경로 비교, 보정 중 OSD/패드 Pause·재개, FPGA 타이밍·PLL 검사를 통과했습니다.
- 배포용 소스의 독립 전체 빌드에서 테스트·배포한 RBF와 SHA256이 완전히 같음을 확인했습니다.
- 동일 RBF의 NA1/NA2 대표 실기 점검과 사용자 CRT 테스트, 일반 설치 파일을 사용한 21개 세트 부팅·어트랙트 점검을 완료했습니다. 새 MAME 픽셀 대조나 오디오 파형 대조를 수행했다는 뜻은 아닙니다.

### 2026-10-10 — OSD/패드 Pause 수정

- OSD 메뉴를 열거나 패드 Pause를 누를 때 VGA 영상 동기가 함께 멈추던 연결을 수정했습니다. 영상 스캔은 계속 유지하고 게임 실행과 사운드 진행은 멈춥니다.
- 패드 Pause 중 OSD를 열고 닫아도 패드의 정지 상태를 유지합니다. 길게 누르면 한 번만 토글되며, Reset과 ROM 로드는 패드 Pause를 해제합니다.
- 실제 코어·68000·CRTC·C219 통합 시뮬레이션, 영상·MCU 주변장치 회귀, 대표 NA1/NA2 실기 정지·재개 검증을 통과했습니다. FPGA 타이밍과 PLL 검사도 통과했습니다.
- 배포용 소스에서 독립 빌드한 RBF와 실기·SD 패키지 RBF의 SHA256이 일치하며, 이 빌드로 21개 세트 부팅·어트랙트 화면을 다시 확인했습니다. 사용자 VGA/OSD 테스트도 통과했습니다.

### 2026-10-08

- **OSD 설정 저장 유지.** 이전에는 `Save settings` 로 저장해도 다음 실행 때 코어가 `.mra` 기본값을 다시 올려 저장값을 덮어썼습니다(MiSTer 는 저장 파일을 ROM 로드 전에 읽고, 코어가 보낸 status 로 128비트 전체를 교체합니다). 이제 저장된 설정이 있으면 기본값을 올리지 않습니다. 실기에서 저장 파일 값이 화면에 반영되는 것을 확인했습니다.

### 날짜별 이정표

- **2026-08 중순** — ROM 구성을 MAME `ROM_START` 와 대조해 검증하고 `.mra` 생성기를 만듦.
  fx68k 를 연결해 68000 이 프로그램 ROM 에서 부팅.
- **2026-08-21** — 실제 M37702 코어가 C70 BIOS 를 실행해 Numan Athletics 가 자가진단을 통과
  (`VERSION PASSED`) 하고 서비스 메뉴를 그림. 그때까지 쓰던 C70 기능 대체 모듈에서 벗어남.
- **2026-08-23** — Emeraldia·Knuckle Heads 를 MAME 와 프레임 단위로 대조. ROZ 레이어와 대체
  스프라이트 팔레트가 첫 노출에서 픽셀 일치. KEYCUS 식별값을 `.mra` 가 내려 주도록 바꿔
  비트스트림 하나로 여러 게임을 지원.
- **2026-08-24** — Quiztou 가 빈 EEPROM 에서 첫 부팅 시 ADDRESS ERROR 로 죽는 원인 규명:
  코어가 아니라 **게임 ROM 자체의 버그**(서브루틴이 D7 을 보존하지 않음)이며 MAME 도 진짜
  빈 EEPROM 에서는 똑같이 죽음. `.mra` 의 EEPROM 초기값으로 해결.
- **2026-08-26** — NA-1 (C69) 13개 세트 전부 실기 동작, 전부 MAME 와 픽셀 일치. C69 는 새
  칩이 아니라 같은 M37702 에 다른 BIOS 였음이 확인됨. 추가 RTL 은 Tinkle Pit 의 KEYCUS 하나.
- **2026-08-27** — 보드 하드웨어(`rtl/`), 중립 어댑터(`integration/`), MiSTer 타깃을 분리.
  세 단계 모두 RBF SHA256 이 이전과 동일함을 확인. 경고 정리.
- **2026-09-09** — 21개 세트 실기 회귀: 부팅, 렌더러 픽셀 일치, 사운드 voice 활동 전부 확인.
  OSD 메뉴를 표준 구성으로 정리(화면비·스캔더블러·DIP·Pause 를 첫 화면에).
- **2026-09-14** — 68000 버스 wait state 수정과 그래픽 꺼짐 구간의 배경색 유지 (아래).
  21개 세트 회귀에서 렌더링 결함 없음.

### 블록별 구현

- **68000** — fx68k 를 쓰되 버스 위상은 코어가 스케줄링합니다. SDRAM 지연 때문에 거의 모든
  버스 사이클에 wait state 가 붙어 실효 속도가 66~78 % 로 떨어지고 있었고, 그 결과 lag
  프레임이 MAME 의 1.4~1.8 배, 어트랙트의 페이드 단계가 빠지는 현상이 있었습니다. DTACK 가
  늦을 때 CPU 위상을 멈췄다가 멈춘 만큼만 되갚는 방식으로 고쳐 lag 비율이 MAME 대비
  0.96~1.07 배가 됐습니다. CPU 클럭을 올린 것이 아니며 OSD Debug 페이지에서 이전 동작으로
  되돌릴 수 있습니다.
- **MCU (C69/C70)** — 공개된 유일한 7700 계열 코어를 가져와 고쳤습니다. 타이머 인터럽트 4종
  추가(BIOS 의 60 Hz 하트비트는 vblank 가 아니라 온칩 Timer A0 입니다), `(sr,S),Y` 와 long
  indirect `[dp]` / `[dp],Y` 주소 모드 추가, `JSR/JMP (abs,X)` 가 인덱스를 두 번 더하던
  업스트림 버그 수정, 0x42·0x89 프리픽스 페이지의 결함 8개 수정. 같은 BIOS 를 MAME 와 나란히
  돌려 **144,534 명령어를 레지스터 단위까지 대조해 불일치 0** 입니다.
- **68000 ↔ MCU 메일박스** — MCU 가 명령 7 에 응답하려면 메일박스 슬롯 2 의 doorbell 비트가
  필요하고, 공유 RAM 창의 16비트 접근은 MAME 의 모델과 바이트 순서가 다릅니다. 둘 다 맞춘
  뒤에야 게임이 버전 확인을 통과했습니다. 홀수 주소 16비트 읽기가 사운드 드라이버에 엉뚱한
  값을 넘겨 BGM 멜로디 voice 가 전부 감쇠되던 결함도 여기서 나왔습니다.
- **C219 사운드** — 16 voice, mu-law, 노이즈, 반전된 왼쪽 채널, 선형 보간, 16비트 랩어라운드
  믹서. `c140.cpp` 를 읽은 해석이 아니라 MAME 가 실제로 낸 어트랙트 오디오와 대조해 차이 0
  (모듈 머리말 기준 219,619 샘플). 샘플을 work RAM 에서 읽으므로
  68000·MCU·블리터와 메모리를 중재합니다.
- **비디오** — 블리터, 타일맵, 스프라이트(셰도 테이블 포함), ROZ, 픽셀 라인 모드를 각각
  MAME 동작의 골든 모델과 대조하고 결함을 일부러 심어 테스트벤치가 잡는지(뮤테이션 테스트)
  확인했습니다. 그다음 실행 중인 MAME 의 비디오 메모리를 꺼내 실제 RTL 로 렌더링해 MAME 의
  그림과 RGB 로 비교했습니다. 그래픽이 꺼진 동안 게임이 배경색에 잠깐 쓰는 값 때문에 연두색
  띠가 번쩍이던 현상은 MAME 처럼 배경색을 유지하도록 고쳤습니다.
- **KEYCUS** — 대부분의 칩은 식별 상수 + 난수입니다. MAME 의 `machine().rand()` 자리에는
  LFSR 을 넣었습니다. Tinkle Pit 의 C367 은 알고리즘으로 답하며, MAME 의 알고리즘과 FBNeo 의
  32개 출력 표가 서로 일치하는 것을 확인한 뒤 구현했습니다.
- **EEPROM / NVRAM** — 28C16 을 구현하고 MiSTer NVRAM 경로로 저장·복원합니다.
- **회전** — Fighter & Attacker / F/A 는 세로 화면 게임이며 OSD 의 Rotate 로 돌려 볼 수
  있습니다. 기판이 실제로 내보내는 것은 회전하지 않은 화면입니다.

### 하드웨어 순도

코어로 들어가는 신호는 클럭, 리셋, 메모리 핸드셰이크, ROM 바이트, 입력 바이트, 설정 상수뿐이며
외부(HPS)가 계산한 결과를 넣는 경로는 없습니다. 게임에 보이는 모든 값은 FPGA 안에서 만들어집니다.

- **2026-10-06** — 라인당 스프라이트 상한을 64 에서 128 로 올렸습니다. MAME 메모리로 돌린
  시뮬레이션에서 잘리던 Emeraldia·Tinkle Pit 장면이 MAME 와 픽셀 단위로 같아졌습니다. EEPROM 을
  모든 세트에서 빈 상태로 시작하게 바꿨고(15개 세트 실기 확인), 21개 세트 모두 실기에서 어트랙트까지 확인했습니다.

## 감사의 말과 사용한 코드

이 코어는 **MAME 팀**의 수십 년에 걸친 작업 없이는 불가능했습니다. NA-1/NA-2 드라이버와
M37710 CPU, C140/C219 사운드 코어에 담긴 지식이 이 RTL 의 사양서였고, 실행 중인 MAME 의
화면·오디오·명령어 흐름이 모든 검증의 기준이었습니다. 깊이 감사드립니다.
— https://www.mamedev.org/ · https://github.com/mamedev/mame

### 함께 빌드되는 외부 코드

| 이름 | 용도 | 저자 | 라이선스 | 출처 (링크 + 커밋) | 사용한 파일 | 수정 여부 |
|---|---|---|---|---|---|---|
| fx68k | 메인 CPU (68000), 사이클 정확 | Jorge Cwik, © 2018, 2021 | GPL-3.0 | https://github.com/ijor/fx68k @ `0602ee4627b10f301298f2673d826cdd6baa9327` | `third_party/cpu/fx68k/` — `fx68k.sv`, `fx68kAlu.sv`, `uaddrPla.sv`, `microrom.mem`, `nanorom.mem` | `fx68k.sv` 만 수정: 읽기 전용 디버그 포트 `dbg_d7` 추가 (동작 변경 없음, 파일 안에 `// LOCAL:` 표시). 나머지 원본 그대로 |
| M37702 CPU 코어 (Namco C76 용) | MCU C69/C70 의 CPU | XelaNotPu | GPL-3.0-or-later (원 저장소 README 의 License 절이 코어 전체를 GPLv3-or-later 로 밝히고, C76 M37702 재구현의 저자를 XelaNotPu 로 적고 있습니다. 파일 자체에는 표기가 없습니다) | https://github.com/XelaNotPu/SYSTEM11_MiSTer @ `b82ac533ef6fa100730f82520f021318f88f5f7e`, `rtl/m37702.vhd` (원본 1,439줄 중 97 % 를 그대로 쓰고 수정·추가) | `projects/namco/na1_na2/rtl/mcu/na2_m37702_cpu.vhd` | 수정함: 이름 변경, 타이머 인터럽트 4종, `bus_word` 출력, 주소 모드 추가, 업스트림 버그 수정 등. 각 지점에 `NA2:` 표시, 파일 머리말에 전체 목록 |
| MiSTer framework (Template_MiSTer) | sys_top, hps_io, OSD, 스케일러, HDMI/VGA, 오디오 출력 | Alexey Melnikov (Sorgelig), Till Harbaum 외 파일별 기여자 | 파일별 GPL-2.0-or-later / GPL-3.0-or-later | https://github.com/MiSTer-devel/Template_MiSTer @ `54ac838e019d7fa07fbb40677a104cd6620d15c3` (2026-08-17, `sys/` 내용 일치로 식별) | `platforms/mister/sys/`, 코어 PLL `targets/mister/rtl/pll*` | `sys/sys.tcl` 의 경로 두 줄만 수정(공유 위치에서 자기 경로를 찾도록). PLL 은 이 코어의 클럭에 맞게 재생성 |

감사합니다 —
**Jorge Cwik** 님, 정확한 68000 을 FPGA 에 공개해 주셔서. 이 코어의 심장입니다.
**XelaNotPu** 님, System 11 코어의 M37702 덕분에 MCU 를 처음부터 만들지 않아도 됐습니다.
**Alexey Melnikov (Sorgelig)** 님과 **MiSTer-devel** 기여자 여러분, 이 플랫폼을 만들고 지켜 주셔서.

EEPROM 은 실제 기판처럼 빈 상태(0xFF)로 전원이 들어옵니다(2026-10-06). 예전에는 Numan
Athletics 의 설정 블록을 모든 세트의 기본값으로 넣었지만 뺐고, 그 파일도 이 배포본에 없습니다.

### 참고한 MAME 소스

MAME 코드는 RTL 에 복사하지 않았습니다. 동작을 읽고 MAME 를 돌려 비교한 뒤 독립적으로
구현했으며, 아래 파일 이름과 줄 번호를 RTL 주석에 근거로 남겼습니다.

| MAME 파일 | 라이선스 | copyright-holders | 참고한 내용 |
|---|---|---|---|
| `src/mame/namco/namcona1.cpp` | BSD-3-Clause | Phil Stroffolino | 메모리 맵, 블리터, KEYCUS, 메일박스, MCU 포트, ROM 정의, 기판 메모 |
| `src/mame/namco/namcona1_v.cpp` | BSD-3-Clause | Phil Stroffolino | 타일맵·스프라이트·ROZ·팔레트·우선순위, 비디오 레지스터 맵 |
| `src/devices/cpu/m37710/m37710.cpp`, `m37710.h` | BSD-3-Clause | R. Belmont, Karl Stenerud | M37702M2 내부 레지스터 맵, 인터럽트 벡터 |
| `src/devices/cpu/m37710/m37710op.h`, `m37710il.h` | BSD-3-Clause | R. Belmont, Karl Stenerud | 0x42/0x89 프리픽스 명령, 주소 모드 의미 |
| `src/devices/sound/c140.cpp` | BSD-3-Clause | R. Belmont | C219 레지스터, 키온, 뱅킹, mu-law, 믹서 |
| `src/emu/tilemap.cpp` | BSD-3-Clause | Aaron Giles | ROZ 샘플링(`draw_roz`)과 스크롤 의미 |

**FinalBurn Neo** (`d_namcona1.cpp`) 는 사실 확인용 교차 대조로만 읽었습니다 — 예를 들어
ROZ 필드 해석과 Tinkle Pit KEYCUS 출력 표. **FBNeo 코드는 한 줄도 가져오지 않았습니다.**
`.mra` 의 interleave 의미는 **mra-tools-c** 의 소스를 읽어 확인했고 코드는 가져오지 않았습니다.

## 라이선스

이 코어 전체는 **GNU General Public License v3.0** 으로 배포됩니다. fx68k(GPL-3.0)가 함께
컴파일되기 때문입니다. 저장소의 `LICENSE` 는 GPL-3.0 전문입니다. 외부 코드 파일은 각자의
머리말과 원래 라이선스 표기를 그대로 유지합니다 (MiSTer framework 는 파일별 GPL-2.0+/3.0+,
M37702 코어는 원 저장소 기준 GPL-3.0-or-later). 참고한 MAME 파일은 모두
BSD-3-Clause 이며 위 표에 출처를 밝혔습니다. 해당 조건·면책 전문은
[`LICENSES/MAME-BSD-3-Clause.txt`](LICENSES/MAME-BSD-3-Clause.txt)에 함께 제공합니다.

**ROM 은 포함되어 있지 않습니다.** 게임 ROM 과 MCU BIOS(`c69.bin`, `c70.bin`)는 각자 합법적으로
소유한 것을 사용해야 합니다. `.mra` 에는 ROM 파일 이름·CRC, 로더·DIP 설정과
아래 제한사항에 설명한 일부 세트의 operator 초기 설정이 들어 있습니다.

## 알려진 제한사항

- **사운드는 샘플 단위로 검증되지 않았습니다.** 21개 세트 voice 활동 계측과 Numan Athletics
  청취 확인까지입니다.
- **라인당 스프라이트 128개 상한** (2026-10-06, 64 에서 올림). 지원 세트가 실제로 요구하는 최대는
  112개(Tinkle Pit)라 지금은 잘리지 않습니다. 실기의 한계는 알려져 있지 않아 근사값입니다.
- **MCU 가 실제 칩보다 느립니다** (측정상 약 1.84배). 어트랙트와 플레이에서 병목은 아니지만
  명령어 타이밍은 사이클 정확이 아닙니다.
- **Tinkle Pit / Emeraldia (Japan, Japan Version B)** 는 게임이 명령 7 에서 C70 BIOS 버전
  문자열을 기대하지만 `.mra` 는 C69 BIOS 를 싣습니다. MAME 는 이 문자열을 하드코딩해 통과합니다.
  실기 동작에는 문제가 없었습니다.
- **EEPROM 초기값.** 대부분의 세트는 빈 EEPROM 에서 스스로 설정을 채웁니다. Cosmo Gang (2),
  Exvania (2), Knuckle Heads (Prototype), Quiztou 6개 세트는 빈 EEPROM 의 첫 부팅에서 멈추는 문제가
  남아 있어 `.mra` 가 MAME 의 초기 EEPROM 내용을 함께 넣습니다. NVRAM 저장은 게임이 EEPROM 을 다시 쓸 때만 일어나고, MiSTer 는 OSD 를
  열 때 저장 요청을 확인합니다.
- **xday2, zelos, zelost, bbbingo, bbbingot 미지원** (위 참조).
- **Analogue Pocket 은 이 릴리스에 포함되지 않습니다.** 부팅은 하지만 ROZ 화면 일부 라인이
  깨지는 미해결 결함이 있습니다.
- MAME 대조는 MAME 0.289 기준입니다.

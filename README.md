# Winch v8.8 (22/56) — Fusion 회전체 생성 스크립트

Autodesk Fusion에서 윈치(winch) v8.8의 회전체(기어, 축, 드럼)를 자동으로 만드는 Python 스크립트입니다.

## 파일 구성

| 경로 | 설명 |
|---|---|
| `winch_bodies_fusion_v8/winch_bodies_fusion_v8.py` | 윈치 회전체를 만드는 Fusion 스크립트 |

## 실행 방법

1. `winch_bodies_fusion_v8` 폴더를 통째로 복사합니다. Fusion 스크립트는 폴더 이름과 파일 이름이 같아야 합니다.
2. Fusion에서 빈 디자인을 엽니다.
3. **Utilities > Add-Ins > Scripts and Add-Ins**에서 스크립트를 추가하고 실행합니다.
4. 실행이 끝나면 **Inspect > Interference**로 맞물리는 부품끼리 간섭이 없는지 확인합니다.

단위는 mm / deg이고, X축이 샤프트 축입니다(0 = W2 안쪽 면).

## 주요 사양

- **1단:** 헬리컬 17T / 56T, m1.0, β 25°, 전위 +0.4 / -0.4
- **2~7단:** 스퍼 23T / 55T, m1.0, 폭 5 mm
- **8단:** 스퍼, 폭 6 mm
- **9단:** 헤링본 m1.5 14T / 35T, β 19.56°, 축간거리 39 mm
- **드럼:** 코어 D13, 평홈 X 73.8–82.0, 45° 콘 X 82–85
- **베어링:** 6700ZZ ×10 (포켓 D15.1)
- **모터축:** (Y 68.8, Z 15.28), 축간거리 40.273

## 설정

스크립트 맨 위 `SETTINGS`에서 아래 항목을 바꿀 수 있습니다.

- `MAKE_BORES`: 중심 구멍 생성
- `SIMPLE_BORE_D`: 계단 보어 대신 단일 직경 구멍
- `MAKE_OTHER_CUTS`: CG8a 육각 소켓, OUT 코드 구멍
- `THIN`: 치형 얇게 하기(백래시)

자세한 변경 이력은 스크립트 docstring에 있습니다.

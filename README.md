# 인텔 엣지 AI 최종 프로젝트 5팀
## 투게더 v1 — 프로젝트 설명
군집 활동을 통한 자율주행 경작지 관리 로봇

## 팀원 소개
![page-02](doc/ppt/투게더v1_pages/page-02.png)
- 김재용(팔로워), 오민지(로봇팔), 황진영(리더), 윤치영(PM) 역할.

## 개요: 선정 배경
![page-04](doc/ppt/투게더v1_pages/page-04.png)
- 농업 자동화 필요성(고령화·생산성 저하)에 대한 문제의식.

## 개요: 목표
![page-05](doc/ppt/투게더v1_pages/page-05.png)
- 군집 로봇을 통한 경작지 순찰·점검·작업의 자율화 목표 설정.

## 개요: 일정
![page-06](doc/ppt/투게더v1_pages/page-06.png)
- 8월~10월 간트차트(메인/서브 로봇, 군집, 로봇팔, 통합, 센서).

## 수행 과정: 구성도
![page-07](doc/ppt/투게더v1_pages/page-07.png)
- 리더(와플)·팔로워(버거), 로봇팔·STM32·거리센서 기반 시스템 구성.

## 리더 로봇: Patrol 개요
![page-08](doc/ppt/투게더v1_pages/page-08.png)
- Nav2 기반 웨이포인트 순찰, 지정 지점 방문 및 모니터링.

## 리더 로봇: 주요 역할
![page-09](doc/ppt/투게더v1_pages/page-09.png)
- 경로/목표 전송, 상태 모니터링, 루프 제어, 복구 처리.

## 리더 로봇: 구성 요소
![page-10](doc/ppt/투게더v1_pages/page-10.png)
- 토픽·액션·노드·파라미터·제어·런치 파일 구성 요약.

## 팔로워 로봇: 개요
![page-11](doc/ppt/투게더v1_pages/page-11.png)
- 리더 상태 구독→상대 위치 계산→추종 제어의 기본 흐름.

## 팔로워 로봇: 주요 역할
![page-12](doc/ppt/투게더v1_pages/page-12.png)
- 리더 정보 수신, 거리/각도 오차 계산, 추종·안전 제어, 센서 피드백.

## 팔로워 로봇: 구성 요소
![page-13](doc/ppt/투게더v1_pages/page-13.png)
- 토픽/노드/메시지/제어/센서/런치 구성.

## 수행 과정: 시퀀스 다이어그램
![page-14](doc/ppt/투게더v1_pages/page-14.png)
- 리더–팔로워 통신·제어 흐름을 시퀀스로 표현.

## 수행 과정: 로봇 경로
![page-15](doc/ppt/투게더v1_pages/page-15.png)
- 맵 상 경로/웨이포인트 예시 및 운행 모습.

## 수행 과정: 로봇 팔
![page-16](doc/ppt/투게더v1_pages/page-16.png)
- STM32 + UART DMA(IDLE) 기반 로봇팔 제어 개요.

## 수행 결과: 사진
![page-17](doc/ppt/투게더v1_pages/page-17.png)
- 구축한 시스템 및 실험 환경 사진.

## 구현 영상
![page-18](doc/ppt/투게더v1_pages/page-18.png)
- 데모 하이라이트 (아래 미리보기 클릭)

[![시연 영상 보기](https://img.youtube.com/vi/ci4-JwFuKa0/hqdefault.jpg)](https://youtu.be/ci4-JwFuKa0)

## 문제 해결: 리더(Nav2)
![page-19](doc/ppt/투게더v1_pages/page-19.png)
- 지그재그/재계획 문제 → Patrol 기반 웨이포인트 운용으로 안정화.

## 문제 해결: 좌표 오차
![page-20](doc/ppt/투게더v1_pages/page-20.png)
- 시뮬 vs 실제 좌표 불일치 → 현장 재추출 좌표로 운용.

## 문제 해결: TF2 오차
![page-21](doc/ppt/투게더v1_pages/page-21.png)
- 좌표계 오차 완화 → 초기 거리 기반 추종으로 단순화.

## 문제 해결: 방향 이슈
![page-22](doc/ppt/투게더v1_pages/page-22.png)
- 회전 시 Following 일시 중지, 서비스 신호로 재정렬 처리.

## 문제 해결: 수평 대형 충돌
![page-23](doc/ppt/투게더v1_pages/page-23.png)
- 수평 대형 충돌 → 수직선상 대형 유지 전략으로 전환.

## 문제 해결: 군집 방식 전환
![page-24](doc/ppt/투게더v1_pages/page-24.png)
- Follower 기반 충돌/겹침 → Patrol 기반 군집으로 안정성 확보.

## 문제 해결: 로봇 팔(토크/전력)
![page-25](doc/ppt/투게더v1_pages/page-25.png)
- AD002 한계 → MG996R 교체, 외부 전원 분리·공급, 자세 재조정.

## 문제 해결: 로봇 팔(추가 보완)
![page-26](doc/ppt/투게더v1_pages/page-26.png)
- 통신/기구 정합성 등 세부 보완 사항 정리.

## 소감: 김재용
![page-27](doc/ppt/투게더v1_pages/page-27.png)
- 문제 접근법·검증 관점 확립, 임베디드 개발자로 성장.

## 소감: 오민지
![page-28](doc/ppt/투게더v1_pages/page-28.png)
- 임베디드 및 AI 실무형 학습, 시스템 관점의 안정성 고려.

## 소감: 황진영
![page-29](doc/ppt/투게더v1_pages/page-29.png)
- 자율주행·군집 구현 경험, 문제 정의→검증→개선 사이클 체득.

## 소감: 윤치영
![page-30](doc/ppt/투게더v1_pages/page-30.png)
- PM 경험(분담·일정·의사결정), BSP/임베디드 이해 심화.

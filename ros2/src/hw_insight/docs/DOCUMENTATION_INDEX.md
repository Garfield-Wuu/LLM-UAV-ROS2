# `hw_insight` 文档地图与维护约定

## 1. 四份核心文档是否都要？

**建议保留四份**，差别在读者与信息类型；要避免的是「同一套启动命令在多处全文复制」。

| 文档 | 首要读者 | 应写什么 | 尽量别写什么 |
|------|----------|----------|----------------|
| [`PRD_text_command_flight_mvp.md`](../PRD_text_command_flight_mvp.md) | 产品 / 架构 / 路线 | 目标架构、阶段、结论、**§9 当前开发主流程（含 T_YOLO）**、运维摘要（如 §13） | 逐步 FAQ、长篇排障流水账 |
| [`README_text_command_test.md`](../README_text_command_test.md) | 日常上手与联调 | **终端怎么开**、`llm_client` 交互、JSON 示例、FAQ、回归命令 | 与 PRD 重复的架构长文（改为链到 PRD） |
| [`PRODUCT_TEST_FLOW.md`](../PRODUCT_TEST_FLOW.md) | 测试 / 发布门槛 | **完成度矩阵、Stage A–I、Release Gate** | 与 PRD §9 完全重复的拓扑长代码块（写「见 PRD §9」+ 最小命令） |
| [`SESSION_HANDOVER.md`](../SESSION_HANDOVER.md) | 跨会话 / 换人接力 | **变更史、Bug 表、Backlog、文件速查**；每次大改更新文首日期 | 与 PRD §9 重复的「每日启动 SOP」全文（链到 PRD） |

## 2. 单一事实源（减少同步负担）

| 主题 | 权威位置 | 其他文档 |
|------|----------|----------|
| 系统五层架构（无 VINS） | [`SYSTEM_ARCHITECTURE.md`](SYSTEM_ARCHITECTURE.md) | PRD §4.1、README 文首 |
| 当前主流程与 **YOLO 终端命令（T_YOLO）** | PRD **§9** | README / SESSION 仅交叉引用 |
| YOLO × AirSim 联调、深度、`on_query`、RViz 叠图 | [`yolo_world_airsim_online_test.md`](yolo_world_airsim_online_test.md) | PRD §13、README §8.1 指向此文 |
| 动作 JSON 协议 | [`COMMAND_PROTOCOL.md`](../COMMAND_PROTOCOL.md) | README §5.2、测试矩阵 |
| 回归 Runner | README §4.5、`PRODUCT_TEST_FLOW` Stage F | 与 PRD 验收表互补 |

## 3. 维护时检查清单

1. 改默认 launch / 视觉行为（如 `inference_mode`、`publish_target_goal`）时：**至少**更新 PRD §3/§6/§9、`yolo_world_airsim_online_test.md`、必要时 `PRODUCT_TEST_FLOW` Stage G/I。
2. 更新 PRD 版本号时：**同步** README 文首「与 PRD vX 一致」、SESSION 文首「关联 PRD vX」。
3. 大功能合入后：在 SESSION 增加一条「第 N 次会话摘要」，不必把全文再抄一遍。

## 4. 若将来要物理合并（可选，工作量大）

- 将 SESSION 中的 **Bug 全表 / 文件创建清单** 迁到 `docs/history/`，SESSION 只保留 1～2 屏索引。
- 或将 `PRODUCT_TEST_FLOW` 的 Stage 表并入 PRD 附录（PRD 会变长，适合以测试为主线的团队）。

默认仍推荐 **分文件 + 本索引**，改动面最小。

## 5. 附录类文档（写作 / 审计留档）

以下用于**论文、汇报或审计留档**，不要求与每次代码改动同步。若与 **PRD**、**`COMMAND_PROTOCOL.md`** 或当前 launch 默认参数冲突，以 PRD / 协议 / 代码为准。

| 文档 | 用途 |
|------|------|
| [`系统技术实现白皮书.md`](系统技术实现白皮书.md) | 系统实现总述（中文，论文素材） |
| [`第四章_系统关键技术实现审计报告.md`](第四章_系统关键技术实现审计报告.md) | 关键技术审计（中文，论文素材） |
| [`ego_planner_feasibility_report.md`](ego_planner_feasibility_report.md) | EGO-Planner 方案评估 |
| [`integration_log_v1.md`](integration_log_v1.md) | 集成排障流水 |

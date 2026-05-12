# Section 5.6 Discussion Notes

- 综合 `TPA` 表现最优的模型是 `Qwen2.5-7B-Instruct`。
- `JSR` 提升最明显的模型是 `Phi-3.5-mini-Instruct`，增幅为 `62.00` 个百分点。
- `MCR` 提升最明显的模型是 `Phi-3.5-mini-Instruct`，增幅为 `60.00` 个百分点。
- 本轮补充实验关闭了 API 级 `format=json`，并将 baseline 改为严格解析 + 严格校验，因此更能真实反映鲁棒提取算法对结构化可执行性的贡献。
- 如果 `JSR` 提升明显且 `MCR` 同步提升，可在 5.6 中强调：鲁棒提取算法不仅提升了解析成功率，也提升了可执行任务的闭环成功率。
- 如果个别模型 `JSR` 提升大于 `MCR` 提升，应说明：解析恢复只是必要条件，最终闭环仍受轨迹、姿态调整与动作序列合理性影响。

## Table Snapshot

- Qwen2.5-7B-Instruct: JSR 46.0 -> 92.0, TPA 65.22, MCR 65.0 -> 70.0, ASC 3.43
- Llama 3.2-3B: JSR 78.0 -> 98.0, TPA 48.98, MCR 65.0 -> 100.0, ASC 1.85
- Phi-3.5-mini-Instruct: JSR 10.0 -> 72.0, TPA 33.33, MCR 0.0 -> 60.0, ASC 1.33

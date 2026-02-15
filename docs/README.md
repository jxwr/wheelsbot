# 文档索引

本文档汇总了项目所有技术文档的位置和用途。

---

## 📚 快速导航

| 如果你想了解... | 阅读文档 |
|----------------|---------|
| **项目背景和目标** | [../CLAUDE.md](../CLAUDE.md) |
| **操作规则（AI开发必读）** | [../MEM.md](../MEM.md) |
| **如何调试参数** | [PARAMETER_TUNING_GUIDE.md](PARAMETER_TUNING_GUIDE.md) |
| **控制架构设计** | [control_system_design.md](control_system_design.md) |
| **当前项目状态** | [project_memory.md](project_memory.md) |
| **历史架构决策** | [decisions/*.md](decisions/) |

---

## 文档分类

### 核心指南

| 文档 | 说明 | 更新频率 |
|------|------|---------|
| `CLAUDE.md` | 项目架构原则、编码规范、Git工作流 | 随架构演进更新 |
| `MEM.md` | AI开发操作规则、ADR格式要求 |  rarely |
| `project_memory.md` | 当前项目状态、技术债务、约束条件 | 每次迭代更新 |

### 控制与调试

| 文档 | 说明 | 目标读者 |
|------|------|---------|
| `PARAMETER_TUNING_GUIDE.md` | 详细的参数调优流程和问题诊断 | 机器人操作员 |
| `control_system_design.md` | 级联控制架构设计说明 | 控制工程师 |
| `DEBUG_SYSTEM_DESIGN.md` | 调试系统技术方案（含CLI工具设计）| 开发者 |
| `DEBUG_SYSTEM_AGENT.md` | AI调试助手集成方案 | AI开发者 |

### 部署与运维

| 文档 | 说明 |
|------|------|
| `deploy.md` | 固件部署指南 |

### 架构决策记录 (ADR)

位于 `decisions/` 目录，每个文件记录一个重要决策：

| ADR文件 | 决策内容 | 状态 |
|---------|---------|------|
| `2026-02-12-cascade-control-separation.md` | 分离级联控制环 | ✅ 已实现 |
| `2026-02-12-migrate-to-cascade-framework.md` | 迁移到级联框架 | ✅ 已完成 |
| `2026-02-12-hardware-abstraction-layer.md` | HAL抽象层设计 | ✅ 已实现 |
| `2026-02-12-cascade-controller-enhancements.md` | 控制器增强 | ✅ 已实现 |
| `2026-02-12-cascade-controller-safety-fixes.md` | 安全修复 | ✅ 已实现 |
| `2026-02-12-cascade-frequency-and-architecture-review.md` | 频率和架构评审 | ✅ 已评审 |
| `2026-02-12-parameter-persistence-design.md` | 参数持久化设计 | ✅ 已实现 |

---

## 历史归档

以下文档保留供参考，但内容已过时：

| 文档 | 说明 | 状态 |
|------|------|------|
| `../CODE_REVIEW_ISSUES.md` | 历史代码审查问题 | ✅ 已修复 |
| `docs/refactors/` | 重构提案历史 | 已归档 |

---

## 更新日志

### 2026-02-15

- 更新 `PARAMETER_TUNING_GUIDE.md` 至 v2.0，匹配级联控制参数
- 更新 `control_system_design.md`，反映当前2层架构
- 更新 `project_memory.md`，添加当前架构说明
- 归档 `CODE_REVIEW_ISSUES.md`，标记问题已修复
- 删除已完成的重构提案文件
- 创建本文档索引

---

**维护者：** 每次添加新文档时，请更新此索引。

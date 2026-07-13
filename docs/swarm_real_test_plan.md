# DroneUmbrella 四机真机测试计划

## 测试顺序

严格按 `Fake -> 单机 -> 两机 -> 四机` 推进。任何阶段失败，都停止进入下一阶段，先记录问题并复测 Fake Swarm。

## 阶段 0：网络与状态

1. 给四台 TT 贴上 `drone_1` 到 `drone_4` 标签。
2. 让四台进入同一局域网，确认实际 IP。
3. 更新 `config.yaml` 的 `swarm.drones`。
4. 运行 `python3 main.py --mode swarm-status`。
5. 确认不打开视频流、不起飞，只读取电量和高度。

验收：四台状态可区分，掉线节点有明确错误。

## 阶段 1：Fake Swarm

```bash
python3 main.py --mode swarm-status --fake
python3 main.py --mode swarm-connect-test --fake
python3 main.py --mode swarm-basic-test --fake
python3 main.py --mode swarm-hover-test --fake
python3 main.py --mode swarm-rc-test --fake
```

验收：单节点失败不崩溃，急停和全体零 RC 有效。

## 阶段 2：单机经 SwarmManager

只在 `config.yaml` 保留一台无人机或只接通一台 TT，运行：

```bash
python3 main.py --mode swarm-status
python3 main.py --mode swarm-connect-test
```

验收：连接、状态、零 RC、stop、land 清理可用，旧单机模式仍可运行。

## 阶段 3：两机起降

只配置两台 TT，空旷环境、保护罩、低电量禁飞：

```bash
python3 main.py --mode swarm-basic-test
python3 main.py --mode swarm-hover-test
```

验收：顺序起飞、悬停、顺序降落。一台异常时执行零 RC 和安全清理。

## 阶段 4：四机起降与悬停

四台全部配置后运行：

```bash
python3 main.py --mode swarm-hover-test
```

验收：四台状态持续可见，统一急停有效，此阶段不发送非零横向指令。

## 阶段 5：四机低速同步移动

确认 `config.yaml` 中 `swarm.rc_test_*` 为低速短时参数后运行：

```bash
python3 main.py --mode swarm-rc-test
```

验收：动作方向一致，短时动作后立即 `zero_rc_all()`，任意节点失联后不继续非零移动。

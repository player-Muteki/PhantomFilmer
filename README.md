# PhantomFilmer 完整真机 WebUI 交付包

本分支是独立的真机 WebUI，包含真机连接、内嵌视频、起飞预检、二次确认起飞与
降落、悬停、手动 RC 控制、失联看门狗、遥测监控、安装脚本和指导书。

首次使用时，在有互联网的网络中运行：

```bash
bash scripts/install_webui.sh
```

安装完成后连接 `RMTT-XXXXXX` 无人机 Wi-Fi，再运行：

```bash
bash scripts/start_webui.sh
```

浏览器访问 `http://127.0.0.1:3000`。完整步骤和故障排查见
[`docs/WebUI真机检查指导书.md`](docs/WebUI真机检查指导书.md)。

本交付包不包含 ReID 或自动跟随模型。

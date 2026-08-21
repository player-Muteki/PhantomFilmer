# PhantomFilmer WebUI 精简交付包

本分支是独立的真机 WebUI 检查包，只包含本次新增或修改的 WebUI、最小真机服务、
安装/启动脚本、测试和指导书，不包含 PhantomFilmer 完整飞控、ReID 或自动跟随源码。

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

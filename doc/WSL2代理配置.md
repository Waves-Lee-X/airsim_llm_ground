# WSL2 使用 Windows 宿主机代理/VPN

> 当 WSL 提示“检测到 localhost 代理配置，但未镜像到 WSL；NAT 模式不支持 localhost 代理”时，含义是 Windows 代理只监听 `127.0.0.1`，而 NAT 模式下的 WSL 是另一台虚拟网络主机。可以让代理监听局域网地址并使用 Windows 网关 IP；若本机 WSL 支持并已启用镜像网络，也可使用其 localhost 转发能力。不要把代理告警误认为 ROS 或 PX4 故障。

## 原理

WSL2 运行在独立的虚拟网络中，通过 NAT 与 Windows 通信。要让 WSL 里的终端走 Windows 上的代理（Clash、V2Ray、SSR 等），需要把 HTTP/HTTPS 请求转发到 Windows 宿主机的代理端口。

```
WSL (Ubuntu)  ──HTTP──▶  Windows 宿主机代理  ──▶  外网
              :port       10.255.255.254:port
```

## 第一步：确认 Windows 代理端口

在 Windows 上打开你的代理软件（Clash / V2RayN / SSR），查看：

- **HTTP 代理端口**：通常是 `7890`（Clash 默认）或 `10809`
- **SOCKS5 端口**：通常是 `7891`（Clash 默认）或 `10808`

## 第二步：确认 Windows 宿主机 IP

WSL2 中执行（每次重启 WSL 可能变化）：

```bash
cat /etc/resolv.conf | grep nameserver | awk '{print $2}'
```

> 通常输出 `10.255.255.254` 或类似值。这个 IP 在 WSL 内部代表 Windows 宿主机。

## 第三步：配置 WSL 代理（临时生效）

```bash
# 把下面两行的端口改成你代理软件的实际端口
export HTTP_PROXY="http://10.255.255.254:7890"
export HTTPS_PROXY="http://10.255.255.254:7890"
export NO_PROXY="localhost,127.0.0.1,::1"

# 验证
curl -I https://www.google.com
```

## 第四步：永久配置

把以下内容加到 `~/.bashrc` 末尾：

```bash
# === WSL2 代理配置 ===
# 自动获取 Windows 宿主机 IP
export WIN_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
export HTTP_PROXY="http://${WIN_IP}:7890"
export HTTPS_PROXY="http://${WIN_IP}:7890"
export NO_PROXY="localhost,127.0.0.1,::1"
```

> **把 `7890` 改成你代理软件的实际 HTTP 端口！**

然后：

```bash
source ~/.bashrc
curl -I https://www.google.com  # 验证
```

## 第五步：让 Git 也走代理

```bash
# 同样把端口改成你自己的
git config --global http.proxy http://10.255.255.254:7890
git config --global https.proxy http://10.255.255.254:7890

# GitHub 不走代理（加速）
git config --global http.https://github.com.proxy ""
```

## 一键开关脚本

保存为 `~/proxy.sh`：

```bash
#!/bin/bash
# WSL2 代理开关脚本

WIN_IP=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}')
PORT=7890  # 改成你的代理端口

proxy_on() {
    export HTTP_PROXY="http://${WIN_IP}:${PORT}"
    export HTTPS_PROXY="http://${WIN_IP}:${PORT}"
    export NO_PROXY="localhost,127.0.0.1,::1"
    echo "代理已开启: ${WIN_IP}:${PORT}"
}

proxy_off() {
    unset HTTP_PROXY
    unset HTTPS_PROXY
    unset NO_PROXY
    echo "代理已关闭"
}

proxy_test() {
    echo "测试 HTTP 代理..."
    curl -sI https://www.google.com | head -1 || echo "代理不通，请检查端口和代理软件状态"
}

# 根据参数执行
case "$1" in
    on)  proxy_on ;;
    off) proxy_off ;;
    test) proxy_test ;;
    *)   echo "用法: source proxy.sh {on|off|test}" ;;
esac
```

用法：

```bash
source ~/proxy.sh on     # 开代理
source ~/proxy.sh off    # 关代理
source ~/proxy.sh test   # 测试
```

> `source` 必须加，否则环境变量只在子 shell 生效。

## 常见问题

### 代理端口不通

```bash
# 先确认 Windows IP 正确
cat /etc/resolv.conf | grep nameserver

# 测端口是否可达
nc -zv $(cat /etc/resolv.conf | grep nameserver | awk '{print $2}') 7890
```

连不上：检查 Windows 防火墙，或者代理软件是否开启了"允许局域网连接"。

### Clash 的 "Allow LAN" 必须打开

Clash 默认只监听 `127.0.0.1`，WSL 从虚拟网络过来无法访问。在 Clash 设置中开启 **Allow LAN**（允许局域网连接），之后会监听 `0.0.0.0:7890`。

### apt / snap 也走代理

```bash
# apt 代理（/etc/apt/apt.conf.d/95proxy）
echo 'Acquire::http::Proxy "http://10.255.255.254:7890";' | sudo tee /etc/apt/apt.conf.d/95proxy

# snap 代理
sudo snap set system proxy.http="http://10.255.255.254:7890"
sudo snap set system proxy.https="http://10.255.255.254:7890"
```

### WSL 重启后 IP 变了

`~/.bashrc` 里的代理配置每次启动自动读取 `/etc/resolv.conf`，IP 变化会自动适配。

### Windows 防火墙拦截

以管理员身份在 PowerShell 执行：

```powershell
New-NetFirewallRule -DisplayName "WSL Proxy" -Direction Inbound -Protocol TCP -LocalPort 7890 -Action Allow
```

import socket
import datetime

def start_debug_server(host='0.0.0.0', port=8000):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server_socket.bind((host, port))
        server_socket.listen(1)
        
        print(f"========================================")
        print(f"✅ [Server] 调试服务器已启动")
        print(f"📍 监听地址: {host}:{port}")
        print(f"Waiting for connection...")
        print(f"========================================", flush=True)

        while True:
            client_socket, client_address = server_socket.accept()
            print(f"\n🔗 [New Connection] 来自: {client_address}", flush=True)
            
            try:
                # 发送欢迎语，证明连接通畅
                welcome_msg = "Server: Connection Accepted;"
                client_socket.sendall(welcome_msg.encode('utf-8'))
                print(f"📤 [Sent Welcome]: {welcome_msg}", flush=True)
                
                while True:
                    # 阻塞接收数据
                    data = client_socket.recv(1024)
                    
                    if not data:
                        print("❌ [Disconnected] 客户端断开连接", flush=True)
                        break
                    
                    # 1. 打印原始字节长度
                    print(f"📥 [Recv {len(data)} bytes]: ", end='', flush=True)
                    
                    try:
                        # 2. 尝试解码并打印
                        text = data.decode('utf-8')
                        # 使用 repr() 可以显示出回车换行符等不可见字符
                        print(f"{repr(text)}", flush=True)
                        
                        # 如果需要看纯文本效果：
                        # print(f"   >>> 内容: {text}", flush=True)
                        
                    except UnicodeDecodeError:
                        print(f"[Binary Data]: {data}", flush=True)

            except ConnectionResetError:
                print("\n⚠️ [Error] 客户端强制关闭了连接", flush=True)
            except Exception as e:
                print(f"\n⚠️ [Error] 通信出错: {e}", flush=True)
            finally:
                client_socket.close()

    except Exception as e:
        print(f"[Fatal Error] 服务器启动失败: {e}")
    finally:
        server_socket.close()

if __name__ == '__main__':
    start_debug_server()
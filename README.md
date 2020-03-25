# Alex

A search and rescue robotic vehicle.

### Enable TLS Server/Client

For running **server** on Raspberry Pi:

```sh
$ cd TLS/server
```
__* If alex.key, alex.crt, and signing.pem are not in the directory, copy them in.__
Then, execute

```sh
$ g++ tls-alex-server.cpp tls_server_lib.cpp tls_pthread.cpp make_tls_server.cpp tls_common_lib.cpp serial.cpp serialize.cpp -pthread -lssl -lcrypto -o tls-alex-server
$ ./tls-alex-server
```

---

For running **client** on the laptop:

```sh
$ cd TLS/client
```

__* Ensure that laptop.key, laptop.crt, and signing.pem are present in the directory.__
Then, execute

```sh
$ g++ tls-alex-client.cpp make_tls_client.cpp tls_client_lib.cpp tls_pthread.cpp tls_common_lib.cpp -pthread -lssl -lcrypto -o tls-alex-client
$ ./tls-alex-client
```

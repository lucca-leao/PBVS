# KUKAVARPROXY

KukavarProxy is a TCP/IP server that enables reading and writing robot variables over the network. ([KUKAVARPROXY](https://github.com/ImtsSrl/KUKAVARPROXY))


## Installation on KRC5 micro

- On the robot SmartPad, login as an Administrator (password 'kuka') and minimize the HMI.

- Copy the contents of this folder to the Windows system on the smartpad (Desktop is fine).

- Copy the file ```cswsk32.ocx``` to ```C:\Windows\SysWOW64```

- On an elevated (Administrator) command prompt, run:

```sh
C:\Windows\SysWOW64\regsvr32   C:\Windows\SysWOW64\cswsk32.ocx
```

- Run ```KukaVarProxy.exe```.

- The server should start running showing the active connections.

- Add a shortcut to the server executable into the Startup folder to run automatically whenever the robot is turned on.

## Third Party
- ([KUKAVARPROXY](https://github.com/ImtsSrl/KUKAVARPROXY))

# Make a copy

> [!TIP]
> Before to start coding sholud be better make a your own copy of this repo in order to make it private and push your code

> [!WARNING]
> Pay attention if you publish any key (such github, Azure, etc keys) on public repository the provider could detect and block it

1. On githb click on create "+" (`Create new...`) located in top right corner.
2. Import repository
3. Paste this repository link: `https://github.com/gdesimone97/cogrob_pepper_nodes.git`
4. Configure it as **private** repository
5. Make the import

# Prerequisites

1. **Docker Engine**: [Installation guide](https://docs.docker.com/engine/install/)
2. **VS Code**: [Installation guide](https://code.visualstudio.com/)
3. **Dev Container Plugin**: [Installation guide](https://code.visualstudio.com/docs/devcontainers/containers)

# How to open enviriorment

1. Clone the repo on your own PC
2. Press `F1` → Select **Build and Reopen in Container**
3. Select the configuration for your OS:
    * WINDOWS
    * MAC
    * Linux
4. Wait for building

# Useful contents
When you run a docker container the filesystem is "simulated" thereby you are working in a virtual enviriorment deteched by filesystem of host machine.
In this context, you can link some directories among host machine and container, thereby any file that you add or make modification inside are modified both on host and container.

In this context, when you run the container following the istructions inside this repository (see [How to open enviriorment](#how-to-open-enviriorment)) the working directory is automatically linked inside `/workspace` directory.

> [!CAUTION]
> We you rebuild the container any data that is not in shared directory with host machine (such as `/workspace`) will be lost and the container will return to its initial state


## Useful commands
Open your VS code command menù either:

* Press `F1`
* `Ctrl` + `Maiusc` + `p`

You need two main commands:

* **Build and Reopen in Container:** This will rebuild the specified image and will open it inside VS code
* **Reopen in Container:** This will reopen the previous built container


## How customize your image
How descrived in [How to open enviriorment](#how-to-open-enviriorment) there different `devcontainer.json` differing for your OS.
As example for WINDOWS/MAC you have [devcontainer.json](.devcontainer/windows_mac/devcontainer.json):

```json
{
    "name": "Windows/Mac Development Container",
    "privileged": false,
    "remoteUser": "mivia",
    "build": {
        "dockerfile": "../Dockerfile",
        "args": {
        }
    },
    "workspaceFolder": "/workspace",
    "workspaceMount": "source=${localWorkspaceFolder},target=/workspace,type=bind",
    "customizations": {
        "vscode": {
            "extensions":[
                "ms-azuretools.vscode-docker",
                "ms-python.vscode-pylance",
                "ms-python.python",
                "mhutchie.git-graph",
                "oderwat.indent-rainbow",
                "mintlify.document",
                "tal7aouy.rainbow-bracket",
                "mechatroner.rainbow-csv",
                "ms-vscode.cmake-tools",
                "ms-vscode.live-server",
            ]
        }
    },
    "containerEnv": {
        "DISPLAY": "host.docker.internal:0"
    },
    "runArgs": [
        //"-p", "3389:3389",
    ],
    "mounts": [
    ],
    "postCreateCommand": "echo 'source /workspace/ws/install/setup.bash' >> /home/mivia/.bashrc",
    "overrideCommand": true
}
```

In general this file will configure the container that you need to use to make code and demo.
It will compile the image by the [Dockerfile](.devcontainer/Dockerfile) that it is empty by default.

If you want install same packages that are permanent in your image you have to add commands within and rebuild the image (see [How to open enviriorment](#how-to-open-enviriorment)).

To install a package you have to write into the file the command to install the package then add `RUN` as prefix, for instance:

```dockerfile
RUN apt update && apt install -y gedit
```

or:

```dockerfile
RUN pip install httpx
```

> [!IMPORTANT]
> Image building it is performed in **no-interactive** mode: you cannot use CLI during the compiling. Thereby if you use `apt` you need to use  `-y` option.
>
> See [Dockerfile.base](.devcontainer/Dockerfile.base) for commands examples

### Note
When you build the container the system will exec the command:
```bash
"echo 'source /workspace/ws/install/setup.bash' >> /home/mivia/.bashrc"
```
In order to avoid to avoid to make source any time you need to run ros2 nodes.
Thereby if you create change/create a new workspace this could create problems, to disable this featues modify the `postCreateCommand"` key inside your target file:

* Windows/Mac: [devcontainer.json](.devcontainer/windows_mac/devcontainer.json)
* Linux: [devcontainer.json](.devcontainer/linux/devcontainer.json)

such as:
```json
"postCreateCommand": ""
```
Then rebuild the container

# How build work space
1. Open container How to open enviriorment (see [How to open enviriorment](#how-to-open-enviriorment)).
2. Go into workspace dir:
```bash
cd ws
```
3. Run building:
```bash
colcon build --symlink-install
```
4. 
```bash
source install/setup.bash
```

# How to run GUI

## X11

### Linux

1. On host machine open a shell and disable *Access Control*
```bash
xhost +
```
2. Open container and run
```bash
xclock
```

### Windows
1. Install [Vcxsrv](https://vcxsrv.com/)
2. Run `XLaunch`
3. Set:
    1. `Multiple windows`
    2. `Display number`: -1
4. Press `Next`
5. Tick `Disable Access Controll`
6. Open container and run
```bash
xclock
```

### MAC
1. Install [XQuartz](https://www.xquartz.org/)
2. Run `XQuartz`
3. Open settings/preferences
5. Disable Access Controll
6. Reboot the system
7. Reopen `XQuartz` by Launchpad
8. Open container and run
```bash
xclock
```

## RDP

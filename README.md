# NetworkGym

## 💡 Overview

## ⌛ Installation
1. Install required Python modules. It is recommended to use a virtual environment with Python version 3.11.
```
pip install -r requirements.txt
```
2. Build the `network_gym_sim`
- Clone this repository.
- Install ns-3.45. In the root directory, clone the [ns-3.45](https://www.nsnam.org/releases/ns-3-45/) and name it as `network_gym_sim`, then clone the [vr-app](https://github.com/ShenMuyuan/ns-3-vr-app) module for generation of burst traffic:
  ```
  git clone -b ns-3.45 https://gitlab.com/nsnam/ns-3-dev.git network_gym_sim
  git clone https://github.com/ShenMuyuan/ns-3-vr-app.git contrib/vr-app
  ```
  After downloading ns-3, install the dependencies and libraries following the [ns-3 prerequisites](https://www.nsnam.org/docs/tutorial/html/getting-started.html#prerequisites). Build the ns-3 with the following commands. You can find more information on building ns-3 [here](https://www.nsnam.org/docs/tutorial/html/getting-started.html#building-ns-3).
  ```
  cd network_gym_sim
  ./ns3 clean
  ./ns3 configure --build-profile=optimized --disable-examples --disable-tests
  ```

- Copy networkgym module files and example programs:
  ```
  cp -r ../network_gym_ns3/contrib/networkgym contrib/
  cp ../network_gym_ns3/scratch/apb.cc scratch/
  cp ../network_gym_ns3/scratch/single-sta-single-link.cc scratch/
  cp ../network_gym_ns3/scratch/multi-bss.cc scratch/
  cp ../network_gym_ns3/scratch/obss.cc scratch/
  cp ../network_gym_ns3/config.txt scratch/
  cp ../network_gym_ns3/network_gym_sim_apb.py .
  cp ../network_gym_ns3/network_gym_sim_ts.py .
  cp ../network_gym_ns3/network_gym_sim_multibss.py .
  cp ../network_gym_ns3/network_gym_sim_obss.py .
  ```
- Install the ZeroMQ socket C++ library (required by networkgym module):
  ```
  sudo apt install libczmq-dev
  ```
  Or on macOS:
  ```
  brew install zeromq cppzmq
  ```
- Add C++ Json library. Replace the `network_gym_sim/contrib/networkgym/model/json.hpp` with the [json.hpp](https://github.com/nlohmann/json/blob/develop/single_include/nlohmann/json.hpp):
  ```
  cd contrib/networkgym/model/
  rm json.hpp
  wget https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp
  cd ../../../scratch
  wget https://raw.githubusercontent.com/nlohmann/json/develop/single_include/nlohmann/json.hpp
  ```

- Try to build ns-3 once again to see if there is any errors:
  ```
  cd ../
  ./ns3 build
  ```

## ☕ Quick Start
First, open 3 terminals (or 3 screen sessions), one per component. Make sure all terminals have activated the virtual environment created in the previous step.
### start server
In the first terminal type following command to start the server:
```
python start_server.py
```
The expected output is as following:
```
Max instances per client:
{'test': 1, 'admin': 100}
┏━━━━━━━━┳━━━━━━━━┳━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┳━━━━━━━━━━━━━┓
┃ Worker ┃ Status ┃ Time since Last Seen (seconds) ┃ Environment ┃
┡━━━━━━━━╇━━━━━━━━╇━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━╇━━━━━━━━━━━━━┩
└────────┴────────┴────────────────────────────────┴─────────────┘
```

### Supported environments

The supported environments are `apb` (A Plus B), `ts` (Thompson Sampling), `multibss` (Multi BSS), and `obss` (Overlapping BSS).

### start environment
In the second terminal type following command to start the ns-3 based environment:
```
python start_env_ns3_<environment-name>.py
```

### start client
In the third terminal, type the following command to start the client:
```
python start_client_<environment-name>.py
```

## 📚 How to reference "NetworkGym"?

Please use the following to reference "NetworkGym" in your paper if it is used to generate data for the paper: 

Menglei Zhang and Jing Zhu, "NetworkGym: Democratizing Network AI via Simulation-as-a-Service", https://github.com/IntelLabs/networkgym 




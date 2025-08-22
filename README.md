# Checkpoint 24 — Task 2
## ROS2 CI for **FastBot Waypoints** with **Jenkins + Docker** (Humble)

This README documents a complete, reproducible **Continuous Integration** setup for **ROS2 (Humble)** that builds a Docker image, launches **Gazebo headless**, starts the **FastBot waypoints action server**, runs **`colcon` gtest**, and reports success/failure in **Jenkins** every time the code changes.

- **Checkpoint**: 24 — *Continuous Integration*  
- **Task**: 2 — Jenkins Basics (automate build & tests for a ROS2 package)  
- **ROS repository under test**: `https://github.com/mathrosas/ros2_testing`  
  - Packages: `fastbot`, `fastbot_waypoints`
  - Tests: C++ (gtest) via `colcon test`

This solution uses a **Freestyle** Jenkins job driven by shell steps and a purpose-built Docker image. No GUI is required during tests (Gazebo runs via **Xvfb**).

---

## Contents

- [What the pipeline does](#what-the-pipeline-does)
- [Prerequisites](#prerequisites)
- [Local quickstart (no Jenkins)](#local-quickstart-no-jenkins)
- [Start Jenkins](#start-jenkins)
- [Create the Jenkins job](#create-the-jenkins-job)
  - [Build steps (copy–paste)](#build-steps-copypaste)
  - [Trigger on Git changes (Poll SCM)](#trigger-on-git-changes-poll-scm)
- [Expected successful output](#expected-successful-output)
- [How the pieces fit together](#how-the-pieces-fit-together)
- [Troubleshooting](#troubleshooting)
- [Optional improvements](#optional-improvements)
- [License](#license)

---

## What the pipeline does

1. **Builds a CI Docker image** from `osrf/ros:humble-desktop`, installing ROS2/Gazebo deps and `python3-colcon-common-extensions`.
2. **Clones** `ros2_testing` and copies only `fastbot` and `fastbot_waypoints` into `/ros2_ws/src/`; **builds** the workspace with `colcon build` during the image build.
3. At runtime, the container **starts Xvfb** (`DISPLAY=:1`), **launches Gazebo** (`fastbot_gazebo/one_fastbot_room.launch.py`), and **starts** the **Waypoints Action Server**.
4. Waits for `/fastbot_as` and `/fastbot/odom`, prints a brief **goal & odom snapshot** (for debugging).
5. **Executes tests**: `colcon test --packages-select fastbot_waypoints` and then prints `colcon test-result`.
6. **Cleans up** processes (action server, gzserver/gzclient, Xvfb) and exits with the **test result code**.
7. **Jenkins** shows pass/fail and stores console logs; it’s triggered automatically by **Poll SCM**.

---

## Prerequisites

- **Ubuntu 22.04+** (or compatible Linux host)  
- **Docker Engine** (and permissions for Jenkins to use it)  
- **Jenkins** (LTS recommended)

Install Docker & enable non-root usage:

```bash
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo service docker start

sudo usermod -aG docker $USER
newgrp docker
```

> CI notes: Allocate sufficient disk/RAM. The run step uses `--shm-size=2g` to keep Gazebo stable in headless mode.

---

## Local quickstart (no Jenkins)

You can build and run the CI container locally to verify everything works before wiring Jenkins.

```bash
# From the directory that contains this README and the Dockerfile
docker build -t fastbot-ros2-ci   --build-arg REPO_URL=https://github.com/mathrosas/ros2_testing.git   --build-arg REPO_BRANCH=main   .

# Run tests headless; entrypoint will start Gazebo + action server + colcon test
docker run --rm --shm-size=2g -e CI=1 fastbot-ros2-ci
```

If the container exits with code `0`, the tests passed.

---

## Start Jenkins

This solution includes **two ways** to start Jenkins.

### Option A — Quick script (if available)

If your environment includes `run_jenkins.sh` (same as Task 1), run:

```bash
bash run_jenkins.sh
```

The script typically:
- Sets `JENKINS_HOME=~/webpage_ws/jenkins/`
- Installs Java (OpenJDK 17)
- Downloads and runs `jenkins.war` in the background
- Prints **PID and URL** to `~/jenkins__pid__url.txt`

Open the printed URL, unlock Jenkins with the initial admin password (see console or `~/webpage_ws/jenkins/secrets/initialAdminPassword`), install suggested plugins, and go to the **Dashboard**.

### Jenkins login (credentials for this checkpoint)

For this exercise, use the following Jenkins admin credentials:

- **Username:** `admin`  
- **Password:** `password`

> ⚠️ **Security note:** These credentials are provided for a learning/lab environment.  
> Do **not** reuse them in public or production systems. Change the password immediately if you expose Jenkins to any untrusted network.

### Option B — Manual commands

```bash
export JENKINS_HOME=~/webpage_ws/jenkins
mkdir -p "$JENKINS_HOME"

sudo apt-get update
sudo apt-get install -y openjdk-17-jre-headless

mkdir -p ~/jenkins_run && cd ~/jenkins_run
wget -q https://get.jenkins.io/war-stable/latest/jenkins.war
java -jar jenkins.war --httpPort=8080
```

---

## Create the Jenkins job

1. **Dashboard → New Item →** *Freestyle project*  
   Name it, for example: **ROS 2 CI – Fastbot Waypoints**.
2. **Description** (optional):  
   “Build Docker, run headless Gazebo + `colcon test` for FastBot waypoints.”
3. **Source Code Management → Git**  
   - Repository URL: `https://github.com/mathrosas/ros2_ci` *(this CI scaffolding repo)*  
   - Branches to build: `main`
4. **Build Triggers**: configure **Poll SCM** (below).  
5. **Build**: add the three shell steps in the next section.

### Build steps (copy–paste)

**Step 1 — Preflight & housekeeping**

```bash
set -euxo pipefail

echo "== Preflight =="

whoami
uname -a
docker --version
git --version
df -h

# allow Jenkins to talk to Docker
sudo chmod 666 /var/run/docker.sock || true

# keep the host clean
docker image prune -f || true
docker container prune -f || true
```

**Step 2 — Build the CI image**

```bash
set -euxo pipefail

docker build --pull -t fastbot-ros2-ci   --build-arg REPO_URL=https://github.com/mathrosas/ros2_testing.git   --build-arg REPO_BRANCH=main   .
```

**Step 3 — Run the tests headless**

```bash
set -euxo pipefail

# Make sure any old container is gone
docker rm -f fastbot-ros2-ci >/dev/null 2>&1 || true

# Run tests (no -it). ENTRYPOINT handles launch + colcon test and exits with the test code.
docker run --name fastbot-ros2-ci --rm   --shm-size=2g   -e CI=1   fastbot-ros2-ci
```

### Trigger on Git changes (Poll SCM)

- **Build Triggers → Poll SCM**: `* * * * *`  
  Jenkins will check the repository every minute and build only if a new commit is found (e.g., after a Pull Request is merged).

---

## Expected successful output

Tail of a passing build (abridged):

```
Successfully tagged fastbot-ros2-ci:latest

+ docker run --name fastbot-ros2-ci --rm --shm-size=2g -e CI=1 fastbot-ros2-ci
[2025-08-21 21:07:52] Starting Xvfb on :1 (headless mode)...
[2025-08-21 21:07:52] Starting fastbot_gazebo...
[2025-08-21 21:07:53] Waiting for Gazebo (/gazebo node or /clock topic)...
[2025-08-21 21:07:53] Starting fastbot action server...
[2025-08-21 21:07:53] Waiting for /fastbot_as action...
[2025-08-21 21:07:54] Waiting for first /fastbot/odom message...
Received first /fastbot/odom
TEST GOAL (from source): goal_x=... goal_y=...
ODOM now: x=... y=... yaw=... deg
Δ to goal: dx=... dy=... dist=... m ; heading_err=... deg
[2025-08-21 21:08:13] Running colcon tests...
Summary: 1 package finished [~1min]
  1 package had test successes: fastbot_waypoints
[2025-08-21 21:09:14] * RESULT: SUCCESS
Finished: SUCCESS
```

---

## How the pieces fit together

- **Dockerfile**
  - Base image: `osrf/ros:humble-desktop`
  - Installs: Gazebo ROS2 pkgs, controllers, `python3-colcon-common-extensions`, and X11/Qt bits for headless rendering.
  - Creates `/ros2_ws`, **clones** `ros2_testing` (branch is parameterized), copies `fastbot` and `fastbot_waypoints` under `/ros2_ws/src`, and **builds** with `colcon build`.
  - Persists env setup to `/root/.bashrc` and sets sane defaults (`ROS_DOMAIN_ID=1`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`).

- **entrypoint.sh**
  - Starts **Xvfb** (`DISPLAY=:1`), launches the **FastBot** world (`one_fastbot_room.launch.py`).
  - Waits for ROS graph readiness, `/fastbot_as`, and `/fastbot/odom`.
  - Prints a **goal & odom snapshot** to aid debugging.
  - Runs **`colcon test --packages-select fastbot_waypoints`** and prints `colcon test-result`.
  - Cleans up the action server and Gazebo; exits with the test code.

- **run_jenkins.sh** (optional helper)
  - Sets `JENKINS_HOME`, installs Java, downloads `jenkins.war`, launches Jenkins, and prints the **URL** & **PID** for convenience.

---

## Troubleshooting

- **“the input device is not a TTY” inside Jenkins**  
  Remove `-it` from `docker run` (this guide’s run step omits it).

- **Docker permissions for Jenkins**  
  If Docker commands fail with permissions errors, the preflight step uses:  
  `sudo chmod 666 /var/run/docker.sock` (fine for a classroom/lab).  
  Prefer adding the Jenkins user to the `docker` group in real deployments.

- **Gazebo headless / display issues**  
  `entrypoint.sh` starts Xvfb and sets `DISPLAY=:1`. Keep `--shm-size=2g` to avoid shared-memory crashes.

- **ROS2 middleware / environment**  
  The image sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and `ROS_DOMAIN_ID=1`. Adjust if your network has DDS conflicts.

- **Disk pressure**  
  Docker images are big; the preflight step prunes images/containers. Periodically `docker system prune -af` if the host is near capacity.

---

## Optional improvements

- **Parameterize the Dockerfile** to fully consume build args:
  ```dockerfile
  ARG REPO_URL="https://github.com/mathrosas/ros2_testing.git"
  ARG REPO_BRANCH="main"
  RUN git clone --branch "${REPO_BRANCH}" --depth 1 "${REPO_URL}" /tmp/ros2_testing
  ```

- **Publish test results** using “Publish JUnit test result report” and point it to:  
  `**/build/**/test_results/**/*.xml` to see test graphs/trends in Jenkins.

- **Enable BuildKit / buildx** to speed up image builds and caching.

- **Switch to a Jenkinsfile** (Pipeline as Code) once the freestyle flow is stable.

---

## License

This CI setup is provided as-is for educational and internal testing purposes.  
Use under this repository’s license and the upstream licenses of ROS packages and `ros2_testing`.

---

### TL;DR (end-to-end)

```bash
# 1) Docker
sudo apt-get update
sudo apt-get install -y docker.io docker-compose
sudo service docker start
sudo usermod -aG docker $USER
newgrp docker

# 2) Jenkins
bash run_jenkins.sh
# Open URL from ~/jenkins__pid__url.txt and complete setup

# 3) Jenkins Job (Freestyle)
#   - SCM: https://github.com/mathrosas/ros2_ci  (branch: main)
#   - Build: paste the three steps from this README
#   - Triggers: Poll SCM (* * * * *)

# 4) Commit & push
# Jenkins builds the image, runs Gazebo headless, executes colcon test, and reports status.
```

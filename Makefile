# Shortcuts. Everything runs inside Docker, so only Docker is needed on the host.
IMAGE := ros2-pid-diffbot
RUN   := docker run --rm -it -v $(CURDIR)/logs:/ws/logs $(IMAGE)

.PHONY: build test wall waypoints plots shell

build:          ## Build the Docker image (ROS 2 Jazzy + Gazebo Harmonic + this workspace)
	docker build -t $(IMAGE) .

test: build     ## Unit tests, closed-loop tests and flake8
	docker run --rm $(IMAGE) bash -c "colcon test --event-handlers console_direct- && colcon test-result --verbose"

wall: build     ## Headless wall-following run, logs written to ./logs
	$(RUN) ros2 launch diffbot_control wall_follow.launch.py headless:=true \
	  log_csv:=/ws/logs/wall_follow.csv trajectory_csv:=/ws/logs/wall_traj.csv

waypoints: build  ## Headless waypoint run, logs written to ./logs
	$(RUN) ros2 launch diffbot_control waypoints.launch.py headless:=true \
	  log_csv:=/ws/logs/waypoints.csv trajectory_csv:=/ws/logs/waypoints_traj.csv

plots:          ## Turn ./logs into the README figures (needs python3 + matplotlib)
	python3 scripts/plot_results.py --logs logs --out docs/media

shell: build    ## Interactive shell inside the container
	$(RUN) bash

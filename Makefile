container_backend=podman
# options for container_backend =
#   docker
#   podman

launch_metrics_backend:
	cd infra && \
	$(container_backend) compose --profile metrics_backend up -d


avionics_file=minerva1.py
# options for avionics_file =
#   dcx_flight_env_expansion.py
#   dcx_hop.py
#   dcx_simple_test.py
#   f9_launch.py
#   hurley_launch.py
#   klipper_launch.py
#   minerva1.py
#   minerva1-krv.py
#   minerva2-reusable.py
#   maxerva1.py
#   maxerva1-cev.py

launch_rocket:
	cd infra && \
	$(container_backend) compose --profile fsw run --build --service-ports fsw uv run $(avionics_file)


stop_all:
	cd infra && \
	$(container_backend) compose --profile fsw down && \
	$(container_backend) compose --profile metrics_backend down
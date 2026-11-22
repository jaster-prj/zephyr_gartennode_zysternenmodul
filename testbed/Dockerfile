FROM ghcr.io/jaster-prj/devcontainer-zephyr:latest

USER root
WORKDIR /workspace

COPY --from=cisternmodule-src . /workspace/application
COPY --from=native-udp-src . /workspace/zephyr-native-linux-udp-can
RUN /opt/venv/bin/west build -b native_sim /workspace/application \
      --build-dir /workspace/application/build/native_sim \
      -- -DZEPHYR_EXTRA_MODULES=/workspace/zephyr-native-linux-udp-can

CMD ["/workspace/application/build/native_sim/zephyr/zephyr.exe"]

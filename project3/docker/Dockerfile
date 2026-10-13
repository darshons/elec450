FROM mambaorg/micromamba:2-ubuntu24.04
USER root
RUN apt-get update && apt-get install -y --no-install-recommends build-essential make git && rm -rf /var/lib/apt/lists/*
USER $MAMBA_USER
RUN micromamba create -y -n ompl_env -c conda-forge python=3.12 ompl cxx-compiler cmake ninja && micromamba clean -a -y
ARG MAMBA_DOCKERFILE_ACTIVATE=1
ENV ENV_NAME=ompl_env
RUN pip install --no-cache-dir nanobind cmake pin numpy pyyaml scipy trimesh yourdfpy "viser>=1.0" matplotlib playwright
USER root
RUN /opt/conda/envs/ompl_env/bin/python -m playwright install-deps chromium && rm -rf /var/lib/apt/lists/*
ENV PLAYWRIGHT_BROWSERS_PATH=/opt/pw
RUN /opt/conda/envs/ompl_env/bin/python -m playwright install chromium && chmod -R a+rx /opt/pw
USER $MAMBA_USER

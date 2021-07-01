FROM continuumio/miniconda3

#ADD environment.yml /zo-asset-tools/environment.yml
RUN conda update -n base -c defaults conda
# RUN conda create --name=zosim_tools python=3.7

# RUN conda install --name zosim_tools -c conda-forge trimesh
# RUN conda install --name zosim_tools -c conda-forge -c dlr-sc -c pythonocc -c oce pythonocc-core==0.17.3
# RUN conda install --name zosim_tools -c conda-forge python-slugify

# RUN conda run -n zosim_tools pip install networkx

RUN conda install -c conda-forge trimesh
RUN conda install -c conda-forge -c dlr-sc -c pythonocc -c oce pythonocc-core==0.17.3
RUN conda install -c conda-forge python-slugify
RUN conda install -c anaconda pip

RUN python -m pip install networkx

RUN  apt-get update \
  && apt-get install -y wget \
  && rm -rf /var/lib/apt/lists/*

# Install graphics library because of pythonocc dependencies
RUN apt-get update -y \
  && apt-get install -y libx11-dev \
  && apt-get install -y libgl1-mesa-glx

# Install custom utilities and libraries
COPY ./docker/tools/vhacd.bash /tmp/
RUN bash /tmp/vhacd.bash

# Copy source code for tools
COPY ./zo_asset_tools/zo_convex_decomposition/ /zo_asset_tools/zo_convex_decomposition/
COPY ./zo_asset_tools/zo_step_to_zosim/ /zo_asset_tools/zo_step_to_zosim/

# Set PYTHOPATH to ZeroSim tools
ENV PYTHONPATH "${PYTHONPATH}:/zo_asset_tools/zo_step_to_zosim:/zo_asset_tools/zo_convex_decomposition"

CMD [ "bash",  "-c", "echo 'Override the entrypoint command when executing this image'"]

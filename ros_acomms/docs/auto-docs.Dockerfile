FROM ros:noetic-ros-core

# Update and upgrade packages, then install common dependencies
RUN apt-get update -qq && \
      apt-get upgrade -y -qq && \
      apt-get install -y -qq \
      python3-pip \
      git \
      graphviz \
      imagemagick \
      make \
      latexmk \
      lmodern \
      fonts-freefont-otf \
      texlive-latex-recommended \
      texlive-latex-extra \
      texlive-fonts-recommended \
      texlive-fonts-extra \
      texlive-lang-cjk \
      texlive-lang-chinese \
      texlive-lang-japanese \
      texlive-luatex \
      texlive-xetex \
      xindy \
      tex-gyre \
      tree && \
      apt-get autoremove -y && \
      apt-get clean && \
      rm -rf /var/lib/apt/lists/*

# Install ROS packages
RUN apt-get update -qq && \
      apt-get upgrade -y -qq && \
      apt-get install -y -qq \
      ros-noetic-dynamic-reconfigure

# Install Python packages
RUN pip install -U sphinx pytest sphinx-book-theme myst_parser && \
      pip install -U git+https://github.com/matteoragni/sphinx_rosmsgs.git@master#egg=sphinx_rosmsgs

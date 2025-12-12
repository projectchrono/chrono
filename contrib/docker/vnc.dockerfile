FROM debian:buster

# Install git, supervisor, VNC, & X11 packages
RUN sed -i 's|deb.debian.org/debian|archive.debian.org/debian|g' /etc/apt/sources.list && \
    sed -i 's|security.debian.org/debian-security|archive.debian.org/debian-security|g' /etc/apt/sources.list && \
    # Buster is archived; disable “Valid-Until” checks
    printf 'Acquire::Check-Valid-Until "false";\nAcquire::Check-Date "false";\n' > /etc/apt/apt.conf.d/99no-check-valid-until && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        bash \
        fluxbox \
        git \
        net-tools \
        novnc \
        supervisor \
        x11vnc \
        xterm \
        xvfb && \
    apt-get clean && apt-get autoremove -y && rm -rf /var/lib/apt/lists/*
    
# Setup demo environment variables
ENV HOME=/root \
    DEBIAN_FRONTEND=noninteractive \
    LANG=en_US.UTF-8 \
    LANGUAGE=en_US.UTF-8 \
    LC_ALL=C.UTF-8 \
    DISPLAY=:0.0 \
    DISPLAY_WIDTH=1728 \
    DISPLAY_HEIGHT=1117 \
    RUN_FLUXBOX=yes \
    RUN_XTERM=no


# noVNC adjustments
# Allows navigation to localhost:8080 instead of localhost:8080/vnc_lite.html
RUN cp /usr/share/novnc/vnc_lite.html /usr/share/novnc/index.html
# Set autoresizing to on
RUN sed -i "/rfb.resizeSession = WebUtil.getConfigVar('resize', false);/a rfb.scaleViewport = true;rfb.resizeSession = true;" /usr/share/novnc/index.html

# Don't want to have to copy over, so just create the supervisord file here
RUN echo "[supervisord]\n\
nodaemon=true\n\
pidfile=/tmp/supervisord.pid\n\
logfile=/dev/fd/1\n\
logfile_maxbytes=0\n\
\n\
[program:x11vnc]\n\
command=x11vnc -forever -shared\n\
autorestart=true\n\
\n\
[program:xvfb]\n\
command=Xvfb :0 -screen 0 '%(ENV_DISPLAY_WIDTH)s'x'%(ENV_DISPLAY_HEIGHT)s'x24 -listen tcp -ac\n\
autorestart=true\n\
\n\
[program:websockify]\n\
command=websockify --web /usr/share/novnc 8080 localhost:5900\n\
autorestart=true\n\
\n\
[program:fluxbox]\n\
command=fluxbox\n\
autorestart=true" > /opt/supervisord.conf

CMD ["sh", "-c", "set -ex; exec supervisord -c /opt/supervisord.conf"]

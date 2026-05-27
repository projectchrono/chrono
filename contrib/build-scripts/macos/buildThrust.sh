#
# Thrust is needed to build the chrono_multicore library on the Mac or any other machine that lacks CUDA
# Note: the thrust cmake files cannot be used!
#
# install wget with homebrew before use
#
INSTALL_DIR=${HOME}/Packages
PACKNAME=thrust-1.17.2
rm -rf ${INSTALL_DIR}/${PACKNAME}
curl -L -o ${PACKNAME}.tar.gz https://github.com/NVIDIA/thrust/archive/refs/tags/1.17.2.tar.gz
tar xvf ${PACKNAME}.tar.gz -C ${INSTALL_DIR}
echo "patching the source for newer versions of clang ...."
sed -i -e '139,144d' ${INSTALL_DIR}/${PACKNAME}/thrust/type_traits/is_contiguous_iterator.h

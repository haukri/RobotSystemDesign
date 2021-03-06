#!/bin/bash
set -e -x

export WORKDIR=$(pwd)
export PYHOME=/home
cd ${PYHOME}

/opt/python/cp27-cp27m/bin/pip2.7 install pip --upgrade
/opt/python/cp35-cp35m/bin/pip3.5 install pip --upgrade
/opt/python/cp36-cp36m/bin/pip3.6 install pip --upgrade
/opt/python/cp37-cp37m/bin/pip3.7 install pip --upgrade

/opt/python/cp37-cp37m/bin/pip install twine cmake
ln -s /opt/python/cp37-cp37m/bin/cmake /usr/bin/cmake

cd ${WORKDIR}

# Collect the pythons
pys=(/opt/python/cp*/bin)

# Filter out Python 3.4 (No numpy support)
pys=(${pys[@]//*34*/})

# Compile wheels
for PYBIN in "${pys[@]}"; do
    "${PYBIN}/pip" install pybind11
    "${PYBIN}/pip" wheel . -w wheelhouse/
    "${PYBIN}/python" setup.py sdist -d wheelhouse/
done

# Bundle external shared libraries into the wheels
for whl in wheelhouse/*$(uname -p).whl; do 
    auditwheel repair "$whl" -w wheelhouse/
    rm $whl
done

ls wheelhouse/

#  Upload
for WHEEL in wheelhouse/ur_rtde*; do
    # dev
    # /opt/python/cp37-cp37m/bin/twine upload \
    #     --skip-existing \
    #     --repository-url https://test.pypi.org/legacy/ \
    #     -u "${K8S_SECRET_TWINE_USERNAME}" -p "${K8S_SECRET_TWINE_PASSWORD}" \
    #     "${WHEEL}"
    # prod
    /opt/python/cp37-cp37m/bin/twine upload \
        --skip-existing \
        -u "${K8S_SECRET_TWINE_USERNAME}" -p "${K8S_SECRET_TWINE_PASSWORD}" \
        "${WHEEL}"
done

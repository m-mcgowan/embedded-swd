
mkdir -p dist

suffix=""

if [ ! -z "$TRAVIS_COMMIT" ]; then
   suffix=_$TRAVIS_COMMIT
fi
if [ ! -z "$TRAVIS_TAG" ]; then
   suffix=_$TRAVIS_TAG
fi

# copy the example to the root so we can build with CLI, replacing include "swd.h" with include "swd/swd.h"
cat firmware/examples/Norwegian_Blue.cpp | sed 's#include "swd/swd.h"#include "swd.h"#g' > firmware/Norwegian_Blue.cpp

for p in photon p1 electron
do
  particle compile $p firmware --saveTo dist/norwegian_blue_$p.bin
done


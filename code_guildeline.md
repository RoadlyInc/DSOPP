# Code Guidelines

We are using [Google C++ Style Guide]

All files including CMake, python, c++ headers/sources should be formatted. There is `format.sh` script to help you with it. 
Run it from project's root as follows:

```
bash fromat.sh
```

### Additional 

#### Template class casts:

* Use ``CastT<NewType>`` alias to cast templated motion/calibration types 

```
using MotionCasted = typename Motion::template CastT<NewScalar>;
using ModelCasted = typename Model::template CastT<NewScalar>;
```

### Pull Request limits

We have soft (300 lines changes) and hard (500 lines changes) limits for pull requests size.

[Google C++ Style Guide]: https://google.github.io/styleguide/cppguide.html


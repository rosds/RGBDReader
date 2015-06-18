#RGBDReader

Read from RGB-D datasets to point cloud structures of the [PCL Library](http://pointclouds.org/).

At the moment it only reads depth images from:

* [ICL-NUIM](http://www.doc.ic.ac.uk/~ahanda/VaFRIC/iclnuim.html)
* [TUM RGB-D](http://vision.in.tum.de/data/datasets/rgbd-dataset)

###Sample programs

The project requires `cmake` for build. To compile simply execute the 
following in the project directory:

```bash
mkdir build
cd build
cmake ..
make
```

The output library should be located in the `lib` directory inside `build`. 
Additionally, you can build some sample binaries with the **WITH_SAMPLES** 
option. Simply specify it when running the `cmake` command inside the `build` 
directory, as:

```bash
cmake -DWITH_SAMPLES=ON ..
```

###Documentation

Doxygen is needed to generate the documentation. Simply execute `make doc` 
inside the `build` directory. The generated documentation should be output 
inside a `doc` directory.

# boids

addon for Godot that adds 2D / 3D boids (flocking).

![boids 2d](./resources/boids_2d.gif)
![boids 3d](./resources/boids_3d.gif)

it can handle about 2000 boids in a single flock at 11ms physics process tick on my PC (Ryzen 5600).
(keep in mind this is without any partitioning of sorts, so it's bound to get better)

## install

download it from the [asset library](https://godotengine.org/asset-library/asset/3284).

or clone the repository, and run `just all` to build the libraries (in release mode) for all supported targets.
(requires [cross](https://github.com/cross-rs/cross), [just](https://github.com/casey/just) and [nushell](https://github.com/nushell/nushell))

currently, linux, windows and web (wasm) is supported.

## usage

take a look at the [examples](./examples/boids/).
the addon folder also contains [a set of default properties extracted from the examples](./addons/boids/defaults/).

## development

it's just a standard rust project under `rust`, so make sure you have `rustup` installed (or the toolchain specified under `rust-toolchain.toml`.)
also don't forget to have godot installed and available in your `PATH` (the extension currently targets 4.3).

- **cargo features**
	- enable `stats` feature to let the extension log into godot console timings for how long its processing the boids.

## todo

- [ ] memoize calculated distances
- [ ] implement avoidance (point avoidance, edge avoidance)
	- [ ] implement nodes for these (for 2d, point and a rect node and 3d point and a cube node, circle / sphere too)
- [ ] implement partitioning (quadtree/octree)
	- [ ] do we just use `spatialtree` crate?
- [ ] write better usage documentation

Bullet classes embedded in Chrono
================

These directory contains a copy of various Bullet files modified for use in Chrono as one option for the collision detection system.

- Chrono-specific changes are marked in various files with comments that contain the string `***CHRONO***`
- To prevent posible linker errors with an existing Bullet installation or when linking to external projects that already use Bullet, all Bullet files and classes used in Chrono were renamed to use the prefix `cbt` (instead of the standard Bullet prefix `bt`).

  This refactoring/renaming can be done automatically using the [repren](https://github.com/jlevy/repren) package, or its Python3 [port](https://github.com/tstapler/repren-1).

  To rename all Bullet files and classes, use a command such as:

  ```shell
  $ ./repren --from="bt([A-Z0-9].*?)" --to='cbt\1' --full .
  ```

  and then remove the `*.orig` files with

  ```shell
  $ find . -name '*.orig' -delete
  ```

  These commands should be issued in the `src/` directory to modify all modules, demos, and tests.

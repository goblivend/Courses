{ pkgs ? import <nixpkgs> {} }:

with pkgs;

mkShell {
  packages = [
    (callPackage ./aarch64-none-elf-gnu.nix {})
  ];
}

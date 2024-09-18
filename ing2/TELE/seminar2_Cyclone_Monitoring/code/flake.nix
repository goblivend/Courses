{ pkgs ? import <nixpkgs> {} }:

with pkgs;

stdenv.mkDerivation {
    pname = "tele";
    version = "1.0.0";

    src = ./.;

    nativeBuildInputs = [
        gcc
        cmake
        ninja
        qt5.wrapQtAppsHook
    ];

    buildInputs = [
        boost
        qt5.full
    ];

    installPhase = ''
        cmake --build .
        mkdir -p $out/bin
        cp tele $out/bin
    '';
}

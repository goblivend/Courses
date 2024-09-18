{
  description = "DISTRO";

  inputs = {
    nixpkgs = {
      type = "github";
      owner = "NixOs";
      repo = "nixpkgs";
      ref = "nixos-22.05";
    };

    flake-utils = {
      type = "github";
      owner = "numtide";
      repo = "flake-utils";
      ref = "main";
    };
  };

  outputs = { self, nixpkgs, flake-utils }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        kas-container = pkgs.stdenv.mkDerivation rec {
          pname = "kas";
          version = "3.1";

          buildInputs = with pkgs; [
            # Kas container requires either podman or docker, it does not
            # support any other container engine.
            podman
            # docker
          ];

          installPhase = ''
            mkdir -p $out/bin
            mv kas-container $out/bin
          '';

          src = pkgs.fetchFromGitHub {
            owner = "siemens";
            repo = pname;
            rev = version;
            hash = "sha256-DivqaaIyLCdAFgC0aklHmMwtGjPASlATggeIT88fF7U=";
          };

        };

      in
      {
        devShell = pkgs.mkShell {
          buildInputs = with pkgs; [
            bmap-tools
            kas-container
          ];
        };

      });
}


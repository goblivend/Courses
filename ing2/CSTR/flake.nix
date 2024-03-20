{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};
    in
    {
      devShells.${system}.default = pkgs.mkShell
        {
          buildInputs = with pkgs; [
            # Fill stuff here
            heptagon
          ];

          C_INCLUDE_PATH = "$C_INCLUDE_PATH:${pkgs.heptagon}/lib/heptagon/c/";
        };
    };
}

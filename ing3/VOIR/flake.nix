{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
  };

  outputs = { self, nixpkgs }:
    let
      system = "x86_64-linux";
      pkgs = nixpkgs.legacyPackages.${system};

      opencvGtk = pkgs.opencv.override (old : { enableGtk2 = true; });
    in
    {
      devShells.${system}.default = pkgs.mkShell
        {
          buildInputs = with pkgs; [
            # Fill stuff here
            opencvGtk
          ];
        };
    };
}

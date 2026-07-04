{
  description = "kayros dev shell (extends nix-dev-base)";

  inputs = {
    base.url = "path:/home/onyr/nix-dev-base";
    nixpkgs.follows = "base/nixpkgs"; # share ONE nixpkgs (less store bloat)
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { base, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config.allowUnfree = true;
        };
      in
      {
        devShells.default = pkgs.mkShell {
          # Base toolkit: python/uv, C/C++ chain, cmake/ninja, dev utils,
          # guarded ./.venv activation.
          inputsFrom = [ base.devShells.${system}.default ];

          packages = with pkgs; [ ];

          shellHook = ''
            echo "[kayros] dev shell active (extends nix-dev-base)."
          '';
        };
      });
}

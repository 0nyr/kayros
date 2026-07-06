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

          packages = with pkgs; [ boost highs ];

          shellHook = ''
            # Vendored Lera BPC (cpp/lera, KAYROS_WITH_LERA=ON builds only):
            # CPLEX 22.1 local install + nixpkgs Boost, same env contract as
            # the TDVRPTW-solver dev shell.
            export CPLEX_HOME=~/cplex2210/CPLEX_Studio221
            export CPLEX_INCLUDE=~/cplex2210/CPLEX_Studio221/cplex/include/
            export CPLEX_BIN=~/cplex2210/CPLEX_Studio221/cplex/lib/x86-64_linux/static_pic/libcplex.a
            # Override the base shell's BAPCOD BOOST_ROOT (1.76) with nixpkgs boost.
            export BOOST_ROOT=${pkgs.boost.dev}
            export BOOST_INCLUDE=${pkgs.boost.dev}/include
            export BOOST_BIN=${pkgs.boost}/lib
            echo "[kayros] dev shell active (extends nix-dev-base; CPLEX/Boost env for KAYROS_WITH_LERA)."
          '';
        };
      });
}

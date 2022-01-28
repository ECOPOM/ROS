{
  inputs = {
    # nixpkgs = { url = "nixpkgs/nixos-20.09"; };
    utils = { url = "github:numtide/flake-utils"; };
  };

  outputs = { self, nixpkgs, utils }: 
  utils.lib.eachDefaultSystem (system:
    let
      pkgs = import nixpkgs.outPath {
        config = { 
          allowUnfree = true;
          allowUnsupportedSystem = true; 
          # cudaSupport = if system == "x86_64-linux" then true else false;
        };
        inherit system;
        overlays = [ ];
      };
      pypkg = pkgs.python38Packages;
    in {
      defaultPackage = pkgs.mkShell {
        name = "cmake";
        buildInputs = let
          opencv = pkgs.opencv.override (old : {
            pythonPackages = pypkg;
            enablePython = true;
            enableGtk3 = true;
            enableGStreamer = true;
            enableFfmpeg = true;
          } );
        in [
          pkgs.libcxxStdenv
          pkgs.libcxx
          pkgs.glfw
          pkgs.fmt
          pkgs.doctest
          pkgs.cmake
          pkgs.ninja
          pkgs.clang_11
          pkgs.clang-tools
          opencv
        ];
        CXX = "clang++";
        CC = "clang";
        shellHook =''
          export LD_LIBRARY_PATH==${pkgs.libcxx}/lib:$LD_LIBRARY_PATH
          export CPLUS_INCLUDE_PATH=$PWD/vendor/vcpkg/installed/x64-linux/include:$CPLUS_INCLUDE_PATH
          export TEST_PATH=${pkgs.libcxxStdenv}
        '';
      };
    }
  );
}

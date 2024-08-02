{
  description = "A devShell example";

  inputs = {
    nixpkgs.url      = "github:NixOS/nixpkgs/nixos-unstable";
    rust-overlay.url = "github:oxalica/rust-overlay";
    flake-utils.url  = "github:numtide/flake-utils";
    rust-toolchain = {
      url = "./rust-toolchain.toml";
      flake = false;
    };
  };

  outputs = { nixpkgs, rust-overlay, flake-utils, rust-toolchain, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        overlays = [ (import rust-overlay) ];
        pkgs = import nixpkgs {
          inherit system overlays;
        };
        toolchain = pkgs.rust-bin.fromRustupToolchainFile rust-toolchain;
        defmt-print = import ./defmt-print.nix {inherit pkgs;};
      in
      with pkgs;
      {
        devShells.default = mkShell {
          buildInputs = [
            toolchain
            espflash

            cargo-binutils
            openocd
            gcc-arm-embedded-10
            defmt-print
          ];
          env.OPENOCD_PATH = "${pkgs.openocd}";
        };
      }
    );
}


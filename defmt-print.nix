{pkgs, ...}:
with pkgs;
    rustPlatform.buildRustPackage rec {
      pname = "defmt";
      version = "0.3.6";
    
      src = pkgs.fetchFromGitHub {
          owner = "knurling-rs";
          repo = pname;
          rev = "defmt-v${version}";
          sha256 = "sha256-8bzl0w33x+MEf6y/5A5HEWxtp/94Ome9i5OP40DDPs8=";
        };
      cargoHash = "sha256-oxSaNR0HPvJKA/pDfuaAQw7sRY0NtO66k7toUoFcxww=";
      doCheck = false;
      cargoPatches = [
        ./add-lock.patch
      ];
      meta = with pkgs.lib; {
        description = "";
        homepage = "";
        changelog = "";
        license = with licenses; [ asl20 /* or */ mit ];
        maintainers = [ ];
      };
    }

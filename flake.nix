{
  description = "Device driver implementations for many embedded sensors and devices";
  inputs = {
    advisory-db = {
      url = "github:rustsec/advisory-db";
      flake = false;
    };

    crane = {
      url = "github:ipetkov/crane";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    devshell = {
      url = "github:numtide/devshell";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-utils.url = "github:numtide/flake-utils";
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";

    pre-commit-hooks = {
      url = "github:cachix/pre-commit-hooks.nix";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };

    rust-overlay = {
      url = "github:oxalica/rust-overlay";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = {
    self,
    advisory-db,
    devshell,
    crane,
    flake-utils,
    nixpkgs,
    pre-commit-hooks,
    rust-overlay,
  } @ inputs:
    flake-utils.lib.eachDefaultSystem (localSystem: let
      pkgs = import nixpkgs {
        inherit localSystem;
        overlays = [
          devshell.overlays.default
          rust-overlay.overlays.default
        ];
      };
      inherit (pkgs) lib;

      rustToolchain = pkgs.pkgsBuildHost.rust-bin.nightly.latest.default;
      craneLib = (crane.mkLib pkgs).overrideToolchain rustToolchain;

      commonArgs = {
        src = craneLib.cleanCargoSource (craneLib.path ./.);
        buildInputs =
          [
            # nothing yet
          ]
          ++ lib.optionals pkgs.stdenv.isDarwin [
            # Additional darwin specific inputs can be set here
            pkgs.libiconv
          ];
      };

      # Build *just* the cargo dependencies, so we can reuse
      # all of that work (e.g. via cachix) when running in CI
      cargoArtifacts = craneLib.buildDepsOnly commonArgs;

      # Build the actual package
      package = craneLib.buildPackage (commonArgs
        // {
          inherit cargoArtifacts;
        });
    in {
      checks =
        {
          # Build the crate as part of `nix flake check` for convenience
          embedded-devices = package;

          # Run clippy (and deny all warnings) on the crate source,
          # again, resuing the dependency artifacts from above.
          #
          # Note that this is done as a separate derivation so that
          # we can block the CI if there are issues here, but not
          # prevent downstream consumers from building our crate by itself.
          embedded-devices-clippy = craneLib.cargoClippy (commonArgs
            // {
              inherit cargoArtifacts;
              cargoClippyExtraArgs = "--all-targets -- --deny warnings";
            });

          embedded-devices-doc = craneLib.cargoDoc (commonArgs
            // {
              inherit cargoArtifacts;
            });

          # Check formatting
          embedded-devices-fmt = craneLib.cargoFmt {
            inherit (commonArgs) src;
          };

          # Audit dependencies
          embedded-devices-audit = craneLib.cargoAudit {
            inherit (commonArgs) src;
            inherit advisory-db;
          };
        }
        // {
          pre-commit = pre-commit-hooks.lib.${localSystem}.run {
            src = ./.;
            hooks = {
              alejandra.enable = true;
              cargo-check.enable = true;
              rustfmt.enable = true;
              statix.enable = true;
            };
          };
        };

      packages.default = package; # `nix build`
      packages.embedded-devices = package; # `nix build .#embedded-devices`

      # `nix develop`
      devShells.default = pkgs.devshell.mkShell {
        name = "embedded-devices";
        imports = [
          "${devshell}/extra/language/c.nix"
          "${devshell}/extra/language/rust.nix"
        ];

        language.rust.enableDefaultToolchain = false;

        commands = [
          {
            package = pkgs.alejandra;
            help = "Format nix code";
          }
          {
            package = pkgs.statix;
            help = "Lint nix code";
          }
          {
            package = pkgs.deadnix;
            help = "Find unused expressions in nix code";
          }
        ];

        devshell.startup.pre-commit.text = self.checks.${localSystem}.pre-commit.shellHook;
        packages = let
          # Do not expose rust's gcc: https://github.com/oxalica/rust-overlay/issues/70
          # Create a wrapper that only exposes $pkg/bin. This prevents pulling in
          # development deps, like python interpreter + $PYTHONPATH, when adding
          # packages to a nix-shell. This is especially important when packages
          # are combined from different nixpkgs versions.
          mkBinOnlyWrapper = pkg:
            pkgs.runCommand "${pkg.pname}-${pkg.version}-bin" {inherit (pkg) meta;} ''
              mkdir -p "$out/bin"
              for bin in "${lib.getBin pkg}/bin/"*; do
                  ln -s "$bin" "$out/bin/"
              done
            '';
        in
          commonArgs.buildInputs
          ++ [
            (mkBinOnlyWrapper rustToolchain)
            pkgs.probe-run
            pkgs.rust-analyzer
          ];
      };

      formatter = pkgs.alejandra; # `nix fmt`
    });
}

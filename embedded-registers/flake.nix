{
  inputs = {
    devshell = {
      url = "github:numtide/devshell";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    flake-parts.url = "github:hercules-ci/flake-parts";
    nci.url = "github:yusdacra/nix-cargo-integration";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    pre-commit-hooks = {
      url = "github:cachix/pre-commit-hooks.nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = inputs:
    inputs.flake-parts.lib.mkFlake {inherit inputs;} {
      imports = [
        inputs.nci.flakeModule
        inputs.pre-commit-hooks.flakeModule
      ];

      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      perSystem = {
        config,
        pkgs,
        ...
      }: let
        projectName = "embedded-registers";
      in {
        pre-commit.settings.hooks = {
          alejandra.enable = true;
          deadnix.enable = true;
          statix.enable = true;
        };

        nci.projects.${projectName}.path = ./.;
        nci.crates.${projectName} = {};

        devShells.default = config.nci.outputs.${projectName}.devShell.overrideAttrs (old: {
          shellHook = ''
            ${old.shellHook or ""}
            ${config.pre-commit.installationScript}
          '';
        });

        packages.default = config.nci.outputs.${projectName}.packages.release;
        formatter = pkgs.alejandra; # `nix fmt`
      };
    };
}

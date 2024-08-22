{
  inputs = {
    devshell = {
      url = "github:numtide/devshell";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
  };

  outputs = inputs:
    inputs.flake-parts.lib.mkFlake {inherit inputs;} {
      imports = [
        inputs.devshell.flakeModule
      ];

      systems = [
        "x86_64-linux"
        "aarch64-linux"
      ];

      perSystem = {
        pkgs,
        system,
        ...
      }: {
        _module.args.pkgs = import inputs.nixpkgs {
          inherit system;
          config.allowUnfree = true;
          config.enableCuda = true;
        };

        devshells.default = {
          packages = [
            (
              pkgs.python3.withPackages (
                p:
                  with p; [
                    torchWithCuda
                    numpy
                    matplotlib
                  ]
              )
            )
          ];
        };

        formatter = pkgs.nixfmt-rfc-style; # `nix fmt`
      };
    };
}

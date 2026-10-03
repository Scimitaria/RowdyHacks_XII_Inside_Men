{
  inputs = {
    nix-ros-overlay.url = "github:lopsided98/nix-ros-overlay/master";
    nixpkgs.follows = "nix-ros-overlay/nixpkgs";
  };

  outputs = { self, nix-ros-overlay, nixpkgs }:
    nix-ros-overlay.inputs.flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [ nix-ros-overlay.overlays.default ];
        };
      in {
        devShells.default = pkgs.mkShell {
          name = "ros-jazzy";
          packages = [
            pkgs.colcon
            (with pkgs.rosPackages.jazzy; buildEnv {
              paths = [
                ros-core
                rmw-cyclonedds-cpp        # the Pi uses Cyclone DDS
                rqt-image-view            # view the robot's camera
                image-transport-plugins   # decode the compressed images
                rviz2                     # optional: odometry and TF
              ];
            })
          ];

          # Same settings as the end of the Pi's ~/.bashrc. Run `nix develop`
          # from this directory so cyclonedds.xml is found.
          shellHook = ''
            export ROS_DOMAIN_ID=77
            export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
            export CYCLONEDDS_URI=file://$PWD/cyclonedds.xml
          '';
        };
      });

  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}

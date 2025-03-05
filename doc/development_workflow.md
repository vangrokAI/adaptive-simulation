# Development Workflow

This document outlines the recommended development workflow for contributing to the Adaptive Surface Contact Simulation project.

## Repository Structure

The project follows a standard ROS2 package structure with the following organization:

```
adaptive-simulation/
├── config/                 # Configuration files for nodes, controllers, etc.
├── doc/                    # Documentation files
├── include/                # C++ header files
│   └── adaptive_simulation/
├── launch/                 # Launch files for various scenarios
├── meshes/                 # 3D model files for visualization
├── msg/                    # Custom message definitions
├── resources/              # Additional resources like images, calibration data
├── scripts/                # Utility scripts for development and testing
├── src/                    # C++ source files
│   ├── ft_twincat_bridge/  # Force/Torque sensor bridge package
│   └── adaptive_control/   # Adaptive control implementation
├── srv/                    # Custom service definitions
├── test/                   # Unit and integration tests
├── urdf/                   # Robot and sensor URDF/XACRO files
├── worlds/                 # Gazebo world definitions
├── CMakeLists.txt          # Build system configuration
├── package.xml             # Package metadata and dependencies
└── README.md               # Project overview
```

## Git Workflow

We follow a feature-branch workflow for development:

1. **Main Branch**: The `main` branch contains the stable, production-ready code
2. **Development Branch**: The `develop` branch contains the latest features being integrated
3. **Feature Branches**: Create feature branches from `develop` for new functionality

### Creating a New Feature

```bash
# Ensure you're on the develop branch
git checkout develop
git pull

# Create a new feature branch
git checkout -b feature/your-feature-name

# Make your changes and commit them
git add .
git commit -m "Descriptive commit message"

# Push your feature branch to the remote repository
git push -u origin feature/your-feature-name
```

### Creating a Pull Request

1. Go to [https://github.com/vangrokAI/adaptive-simulation/pulls](https://github.com/vangrokAI/adaptive-simulation/pulls)
2. Click "New pull request"
3. Select `develop` as the base branch and your feature branch as the compare branch
4. Fill in the pull request template with details about your changes
5. Request a review from a team member

### Code Review Process

All code changes require at least one review from a team member before merging. Reviewers should check for:

1. Code quality and adherence to style guidelines
2. Proper test coverage
3. Documentation
4. Performance implications

## Development Environment

### Recommended Setup

1. **IDE**: Visual Studio Code with ROS2 extension
2. **Terminal**: Terminator or similar for multiple terminal panes
3. **Build System**: colcon

### VS Code Extensions

- C/C++ Extension Pack
- Python
- XML Tools
- YAML
- ROS
- CMake
- Docker (for containerized development)

### Environment Configuration

Create a workspace with both this repository and the LBR stack:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/vangrokAI/adaptive-simulation.git
git clone https://github.com/lbr-stack/lbr_fri_ros2_stack.git

# Build the workspace
cd ~/ros2_ws
colcon build --symlink-install

# Source the workspace
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Testing

### Running Tests

```bash
# Run all tests
cd ~/ros2_ws
colcon test

# Run specific tests
colcon test --packages-select adaptive_simulation

# View test results
colcon test-result --verbose
```

### Writing Tests

1. **Unit Tests**: Place in `test/unit/` directory
2. **Integration Tests**: Place in `test/integration/` directory
3. **System Tests**: Place in `test/system/` directory

All tests should be included in the CMakeLists.txt file.

## Documentation

### Code Documentation

- C++ code should use Doxygen-style comments
- Python code should use docstrings following Google Python Style Guide

### User Documentation

- Update the relevant Markdown files in the `doc/` directory
- Keep the README.md updated with the latest setup instructions

## Building and Releasing

### Local Build

```bash
# Clean build
cd ~/ros2_ws
rm -rf build/ install/
colcon build --symlink-install

# Incremental build
colcon build --symlink-install --packages-select adaptive_simulation
```

### Release Process

1. Update the version number in `package.xml`
2. Create a changelog entry in `CHANGELOG.md`
3. Tag the release in Git:
   ```bash
   git tag -a v1.0.0 -m "Release v1.0.0"
   git push origin v1.0.0
   ```
4. Create a release on GitHub with release notes

## Continuous Integration

The repository uses GitHub Actions for CI/CD:

- Automated building and testing on push to `develop` and `main`
- Linting checks for code quality
- Documentation generation
- Release automation

CI workflow files are located in the `.github/workflows/` directory.

## Troubleshooting

### Common Issues

1. **Build Failures**:
   - Check the build log for specific errors
   - Ensure all dependencies are installed
   - Try a clean build

2. **Runtime Errors**:
   - Check ROS2 logs with `ros2 log`
   - Use `ros2 topic echo` to inspect messages
   - Enable debug output with appropriate parameters

3. **Integration Issues**:
   - Ensure your ROS domain ID is set correctly
   - Check network connectivity for distributed systems
   - Verify correct package and executable paths

### Getting Help

- Create an issue on GitHub with detailed information
- Contact the project maintainers
- Check the existing issues for similar problems

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [KUKA FRI Documentation](https://github.com/kroshu/kuka_fri/wiki)
- [TwinCAT ADS Documentation](https://infosys.beckhoff.com/english.php?content=../content/1033/tcadscommon/html/tcadscommon_intro.htm)
- [ATI Force/Torque Sensor Documentation](https://www.ati-ia.com/products/ft/ft_ModelListing.aspx?category=Axia)

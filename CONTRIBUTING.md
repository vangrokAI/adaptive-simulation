# Contributing to Adaptive Surface Contact Simulation

Thank you for your interest in contributing to the Adaptive Surface Contact Simulation project! This document provides guidelines and instructions for contributing.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Process](#development-process)
- [Pull Request Process](#pull-request-process)
- [Coding Standards](#coding-standards)
- [Testing](#testing)
- [Documentation](#documentation)
- [Issue Reporting](#issue-reporting)

## Code of Conduct

This project adheres to a Code of Conduct that sets expectations for participation in our community. By participating, you are expected to uphold this code. Please report unacceptable behavior to the project maintainers.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** to your local machine
3. **Set up the development environment** as described in the [Development Workflow](./doc/development_workflow.md) document
4. **Create a branch** for your feature or bugfix

## Development Process

1. **Choose an Issue**: Start by finding an open issue to work on or create a new one if you've identified a problem or enhancement.
2. **Discuss**: For significant changes, please open an issue first to discuss what you would like to change. This helps ensure your time is well spent.
3. **Branch**: Create a feature branch from the `develop` branch.
4. **Develop**: Implement your changes, following the coding standards and including tests.
5. **Test**: Run the tests to ensure your changes don't break existing functionality.
6. **Document**: Update documentation as necessary.

## Pull Request Process

1. **Push** your changes to your fork on GitHub
2. **Create a Pull Request** from your branch to the project's `develop` branch
3. **Describe** the changes you've made and reference any related issues
4. **Wait for Review**: Maintainers will review your PR and provide feedback
5. **Address Feedback**: Make any necessary changes based on the review
6. **Merge**: Once approved, a maintainer will merge your PR

Pull requests must meet the following requirements before being merged:
- They must pass all automated tests
- They must be reviewed and approved by at least one maintainer
- They must meet the coding standards
- Documentation must be updated if necessary

## Coding Standards

### C++ Code Style

- Follow the [ROS2 C++ Style Guide](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html)
- Use camelCase for function names, variables, and namespaces
- Use PascalCase for class names
- Use UPPER_CASE for constants and macros
- Include proper Doxygen-style documentation for all public APIs

### Python Code Style

- Follow [PEP 8](https://www.python.org/dev/peps/pep-0008/)
- Use Google-style docstrings
- Maximum line length is 100 characters

### Commit Messages

- Write clear, concise commit messages in the imperative mood (e.g., "Add feature" not "Added feature")
- Reference issue numbers when applicable
- Format: `[Component] Brief description (fixes #123)`

## Testing

- All new features must include appropriate tests
- All bug fixes should include tests that verify the fix
- Run all tests before submitting a PR
- Aim for high test coverage, especially for critical components

## Documentation

- Update the relevant documentation for any user-facing changes
- Document all public APIs with appropriate comments
- Include examples for complex functionality
- Update the README.md if necessary
- For major features, consider creating or updating a dedicated document in the `doc/` directory

## Issue Reporting

When reporting issues, please include:

1. **Steps to Reproduce**: Clear, step-by-step instructions
2. **Expected Result**: What you expected to happen
3. **Actual Result**: What actually happened
4. **Environment**: 
   - OS version
   - ROS2 version
   - Hardware details (if relevant)
   - Output of `ros2 doctor` (if applicable)
5. **Additional Context**: Any other information that might be helpful

## License

By contributing, you agree that your contributions will be licensed under the project's license.

## Questions?

If you have any questions or need help, please:
1. Check the existing documentation
2. Look for similar issues on the issue tracker
3. Create a new issue for questions that might be relevant to others
4. Contact the project maintainers for specific questions

Thank you for contributing to make this project better!

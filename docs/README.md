# CostNav Documentation

This directory contains comprehensive documentation for the CostNav project.

## Documentation Structure

### Getting Started
Start here if you're new to CostNav:

1. **[Quick Reference](quick_reference.md)** - Common commands and code snippets for quick access
2. **[Architecture Overview](architecture.md)** - High-level overview of the codebase structure
3. **[Environment Versions](environment_versions.md)** - Understanding v0, v1, and v2 environments

### Core Concepts
Deep dives into the fundamental components:

1. **[MDP Components](mdp_components.md)** - Detailed explanation of observations, actions, rewards, terminations, commands, and events
2. **[Robot Configuration](robot_configuration.md)** - COCO robot's physical properties, actuators, sensors, and control system
3. **[Cost Model](cost_model.md)** - Business metrics: SLA compliance, operational costs, profitability, and break-even time

### Guides
Step-by-step guides for common tasks:

1. **[Training Guide](training_guide.md)** - Complete guide to training navigation policies with various RL frameworks

### Reference
Detailed reference documentation:

1. **[API Reference](api.md)** - Auto-generated API documentation for all Python modules
2. **[Scripts Reference](scripts_reference.md)** - Comprehensive reference for all training, evaluation, and utility scripts
3. **[Food Delivery Business](references/FOOD_DELIVERY_BUSINESS.md)** - Industry data and economics for food delivery robots

### Project Information

1. **[Contributing](contributing.md)** - Guidelines for contributing to CostNav
2. **[Roadmap](TODO.md)** - Current tasks and future plans

## Building the Documentation

### Local Preview

```bash
# Install MkDocs and dependencies
pip install mkdocs mkdocs-material mkdocstrings[python]

# Serve documentation locally
mkdocs serve

# Open browser to http://127.0.0.1:8000
```

### Build Static Site

```bash
# Build documentation
mkdocs build

# Output will be in site/ directory
```

### Deploy to GitHub Pages

```bash
# Deploy to gh-pages branch
mkdocs gh-deploy
```

## Documentation Guidelines

### Writing Style

- **Clear and Concise**: Use simple language, avoid jargon where possible
- **Code Examples**: Include working code snippets with explanations
- **Visual Aids**: Use diagrams, tables, and code blocks to illustrate concepts
- **Cross-References**: Link to related documentation sections

### Code Snippets

Use syntax highlighting for code blocks:

````markdown
```python
# Python code
def example():
    pass
```

```bash
# Bash commands
python script.py --arg=value
```
````

### Admonitions

Use admonitions for important information:

````markdown
!!! note
    This is a note.

!!! warning
    This is a warning.

!!! tip
    This is a tip.
````

### File Organization

- **Main docs**: Place in `docs/` root
- **Reference docs**: Place in `docs/references/`
- **Images**: Place in `docs/images/`
- **Assets**: Place in `docs/assets/`

## Contributing to Documentation

### Adding New Pages

1. Create new `.md` file in `docs/`
2. Add entry to `mkdocs.yml` navigation
3. Link from relevant existing pages
4. Test locally with `mkdocs serve`

### Updating Existing Pages

1. Edit the `.md` file
2. Test locally with `mkdocs serve`
3. Verify all links work
4. Check formatting and code examples

### API Documentation

API documentation is auto-generated from docstrings using `mkdocstrings`:

```python
def example_function(param1: str, param2: int) -> bool:
    """Short description.
    
    Longer description with more details.
    
    Args:
        param1: Description of param1
        param2: Description of param2
        
    Returns:
        Description of return value
        
    Example:
        ```python
        result = example_function("test", 42)
        ```
    """
    pass
```

## Documentation Checklist

When adding new features, ensure documentation includes:

- [ ] Overview of the feature
- [ ] Code examples showing usage
- [ ] Configuration options
- [ ] Common use cases
- [ ] Troubleshooting tips
- [ ] Links to related documentation
- [ ] API reference (if applicable)

## Questions or Issues?

- **GitHub Issues**: https://github.com/worv-ai/CostNav/issues
- **Discussions**: https://github.com/worv-ai/CostNav/discussions
- **Documentation Issues**: Tag with `documentation` label


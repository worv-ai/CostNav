# :handshake: Contributing to CostNav

We welcome contributions to CostNav! This document provides guidelines for contributing to the project.

---

## :rocket: Quick Start

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes
4. Commit your changes: `git commit -m 'Add amazing feature'`
5. Push to the branch: `git push origin feature/amazing-feature`
6. Open a Pull Request

---

## :memo: Documentation

We use [MkDocs](https://www.mkdocs.org/) with the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme for our documentation.

### :computer: Running Documentation Locally

To run the documentation locally, you need to have `uv` installed.

=== "Using uv"

    ```bash
    # Install dependencies
    uv pip install -e ".[dev]"

    # Serve the documentation
    mkdocs serve
    ```

=== "Using pip"

    ```bash
    # Install dependencies
    pip install mkdocs-material mkdocstrings[python] pymdown-extensions

    # Serve the documentation
    mkdocs serve
    ```

This will start a local server at `http://127.0.0.1:8000/`. The documentation will automatically reload when you make changes.

### :pencil: Writing Documentation

| Location | Content |
|:---------|:--------|
| `docs/` | All documentation files |
| `mkdocs.yml` | Documentation configuration |
| Docstrings | API documentation (auto-generated) |

!!! tip "Markdown Features"
    We support many Markdown extensions including:

    - :white_check_mark: Admonitions (`!!! note`)
    - :white_check_mark: Code tabs (`=== "Tab 1"`)
    - :white_check_mark: Mermaid diagrams (` ```mermaid `)
    - :white_check_mark: Emoji (`:rocket:`)
    - :white_check_mark: Task lists (`- [x] Done`)

---

## :computer: Code Contributions

### :art: Coding Standards

| Aspect | Standard |
|:-------|:---------|
| Formatter | [Ruff](https://docs.astral.sh/ruff/) |
| Linter | [Ruff](https://docs.astral.sh/ruff/) |
| Type hints | Required for public APIs |
| Docstrings | Google style |

### :test_tube: Running Tests

```bash
# Run all tests
uv run pytest

# Run with coverage
uv run pytest --cov=costnav_isaaclab --cov-report=xml
```

### :white_check_mark: Pre-commit Checks

```bash
# Format code
ruff check --fix
ruff format

# Run tests
uv run pytest
```

---

## :bug: Bug Reports

When reporting bugs, please include:

- [ ] :computer: Operating system and version
- [ ] :snake: Python version
- [ ] :robot: Isaac Sim and Isaac Lab versions
- [ ] :page_facing_up: Complete error message/traceback
- [ ] :repeat: Steps to reproduce
- [ ] :crystal_ball: Expected vs actual behavior

---

## :bulb: Feature Requests

When suggesting features, please include:

- [ ] :dart: Clear description of the feature
- [ ] :question: Problem it solves
- [ ] :memo: Example use cases
- [ ] :art: Mockups or diagrams (if applicable)

---

## :shield: Code of Conduct

Please be respectful and constructive in all interactions. We're all here to build something great together!

---

## :pray: Thank You!

Thank you for considering contributing to CostNav. Your contributions help make this project better for everyone!

# Contributing to CostNav

We welcome contributions to CostNav! This document provides guidelines for contributing to the project.

## Documentation

We use [MkDocs](https://www.mkdocs.org/) with the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) theme for our documentation.

### Running Documentation Locally

To run the documentation locally, you need to have `uv` installed.

1.  **Install dependencies**:
    If you haven't already, install the project dependencies including development tools:
    ```bash
    uv pip install -e ".[dev]"
    ```

2.  **Serve the documentation**:
    Run the following command to start a local server:
    ```bash
    mkdocs serve
    ```
    This will start a local server at `http://127.0.0.1:8000/`. The documentation will automatically reload when you make changes.

### Writing Documentation

-   Documentation files are located in the `docs/` directory.
-   We use Markdown for writing documentation.
-   API documentation is automatically generated from docstrings using `mkdocstrings`.

## Code Contributions

(Add general code contribution guidelines here, e.g., coding standards, testing, etc.)

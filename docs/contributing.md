# :handshake: Contributing

Help us build a large-scale, ever-expanding benchmark! We highly encourage contributions via issues and pull requests, especially adding more navigation baselines.

---

## :rocket: Quick Start

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes
4. Run pre-commit checks: `pre-commit run --all-files`
5. Commit and push
6. Open a Pull Request using our [PR template](#pull-request-format)

---

## :white_check_mark: Pre-commit Hooks

The project uses [pre-commit](https://pre-commit.com/) with Ruff for linting and formatting.

```bash
# Install hooks
./scripts/setup-pre-commit.sh

# Run manually
pre-commit run --all-files

# Format code
ruff check --fix
ruff format
```

!!! info "Pre-push Hooks"
    Pre-push hooks run `ruff check` and `ruff format --check` before pushing. If blocked, fix the reported issues and try again.

---

## :page_facing_up: Pull Request Format

Use our PR template with the following title format:

```
[Priority Emoji] [Pattern]: Brief description
```

| Emoji | Meaning |
|:------|:--------|
| :fire: | New feature implementation |
| :rocket: | Urgent fix |
| :test_tube: | Experimental / refactor |
| :memo: | Documentation / configuration |
| :zap: | Cleanup / refactor |

**Examples:**

- `:fire: Add: domain adaptation module with adversarial training`
- `:rocket: Fix: urgent data collection script for demo`
- `:memo: Docs: update README with installation instructions`

### PR Checklist

- [ ] :dart: Purpose: New feature / Bug fix / Docs / Refactoring
- [ ] :repeat: Reproduce: Execution commands included
- [ ] :bar_chart: Changes: Summary of main changes
- [ ] :test_tube: Testing: Verified locally
- [ ] :thinking: Review Focus: Areas where feedback is needed

---

## :memo: Documentation

We use [MkDocs](https://www.mkdocs.org/) with [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/).

```bash
# Install dependencies
pip install mkdocs-material pymdown-extensions

# Serve locally
mkdocs serve
```

| Location | Content |
|:---------|:--------|
| `docs/` | All documentation files |
| `mkdocs.yml` | Documentation configuration |

---

## :dart: Roadmap

### Completed

- [x] Paper release
- [x] Isaac Sim assets release
- [x] Nav2 support for rule-based navigation
- [x] Cost formula and reference sheet
- [x] Collected dataset with teleoperation
- [x] Imitation learning baselines

### Coming Soon

- [ ] Diverse maps and robot platforms
- [ ] Expanded scenarios testing robustness under challenging conditions
- [ ] Cost-aware reward shaping for RL training
- [ ] Cloud inference cost modeling for VLA policies
- [ ] Open challenges for the community to beat baselines

---

## :bug: Bug Reports

When reporting bugs, please include:

- [ ] Operating system and version
- [ ] Python version
- [ ] Isaac Sim version
- [ ] Complete error message / traceback
- [ ] Steps to reproduce
- [ ] Expected vs actual behavior

---

## :bulb: Feature Requests

When suggesting features, please include:

- [ ] Clear description of the feature
- [ ] Problem it solves
- [ ] Example use cases

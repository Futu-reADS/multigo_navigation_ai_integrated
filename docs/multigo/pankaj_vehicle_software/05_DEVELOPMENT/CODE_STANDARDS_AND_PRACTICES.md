# Code Standards and Practices

**All Teams**
**Date:** 2025-12-16
**Version:** 1.0

## General Principles

1. **Write tests first** (TDD where applicable)
2. **Code reviews required** (≥1 approver before merge)
3. **Keep functions small** (<50 lines)
4. **Meaningful names** (no `temp`, `data`, `foo`)
5. **Document complex logic** (comments explain WHY, not WHAT)

## Git Commit Messages

Use Conventional Commits:
```
feat: add ArUco docking controller
fix: correct CAN bus termination issue
docs: update API specification
test: add integration tests for docking
refactor: simplify swerve drive kinematics
```

## Code Review Checklist

- [ ] Tests pass (`colcon test` or `npm test`)
- [ ] Code coverage ≥80%
- [ ] No linting errors
- [ ] Documentation updated
- [ ] No security vulnerabilities (Dependabot, Snyk)

## Testing Requirements

- **Unit tests:** Mock all external dependencies
- **Integration tests:** Test component interactions
- **E2E tests:** Test full workflows (use in CI/CD)

---
**References:** .claude/WORKFLOW.md, TESTING_REQUIREMENTS.md

# Git Subtree Setup Guide

This document explains the step-by-step process for adding individual Pico firmware repositories as subtrees under the `apps/` directory. This workflow allows us to pull in only specific subdirectories from external repositories while maintaining a clean monorepo structure.

## Overview

The process involves:
1. Adding a remote for the source repository
2. Fetching the remote content
3. Creating a local tracking branch
4. Splitting out the specific subdirectory we need
5. Adding the split content as a subtree in our monorepo
6. Cleaning up temporary branches and remotes

## Step-by-Step Process

### 1. Adding the Remote

```bash
git remote add motor-remote git@github.com:EIGSEP/motor-controller.git
```

- Adds a new remote named `motor-remote` pointing to the source repository
- Use descriptive remote names to avoid conflicts when working with multiple repos
- Replace `motor-remote` and the URL with appropriate values for each project

### 2. Fetching the Remote

```bash
git fetch motor-remote
```

- Downloads all branches and commits from the remote repository
- This populates our local git database with the remote repository's history
- Required before we can reference remote branches in subsequent steps

### 3. Creating a Temporary Branch

```bash
git checkout -b motor-local motor-remote/main
```

- Creates a new local branch `motor-local` that tracks `motor-remote/main`
- This gives us a local working branch based on the source repository
- Allows us to work with the remote content in our local repository context
- Replace `main` with the appropriate branch name if the source uses a different default branch

### 4. Splitting Out the Subdirectory

```bash
git subtree split --prefix=pico_c --branch=motor-c-only
```

- `--prefix=pico_c`: Specifies the subdirectory to extract from the source repository
- `--branch=motor-c-only`: Creates a new branch containing only the history of the specified subdirectory
- This command creates a new branch where each commit only contains changes to files in the `pico_c` directory
- The resulting branch has a clean history with only the relevant subdirectory content

### 5. Merging into Our Monorepo

```bash
git checkout main
git subtree add --prefix=apps/motor motor-c-only --squash
```

- `git checkout main`: Switch back to our main branch where we want to add the subtree
- `--prefix=apps/motor`: Specifies where in our repository the subtree content should be placed
- `motor-c-only`: References the split branch we created in the previous step
- `--squash`: Combines all commits from the subtree into a single commit in our repository
  - **Why squash?** Keeps our main repository history clean and focused
  - **Alternative**: Without `--squash`, all individual commits from the source repo would be preserved
  - **Trade-off**: Squashing loses detailed history but maintains repository clarity

### 6. Clean-up Steps

```bash
git branch -d motor-c-only
git branch -d motor-local
git remote remove motor-remote
```

- Delete the temporary split branch (`motor-c-only`) - no longer needed after merge
- Delete the temporary local branch (`motor-local`) - no longer needed
- Remove the temporary remote (`motor-remote`) - keeps our remote list clean
- These steps ensure our repository doesn't accumulate unnecessary references

## Example: Adding switchNW Repository

Here's how to apply the same process for the switchNW repository:

```bash
# 1. Add remote
git remote add switch-remote git@github.com:EIGSEP/switchNW.git

# 2. Fetch remote
git fetch switch-remote

# 3. Create temporary branch (using merge-pico-to-c branch)
git checkout -b switch-local switch-remote/merge-pico-to-c

# 4. Split out pico_c subdirectory
git subtree split --prefix=pico_c --branch=switch-c-only

# 5. Merge into monorepo
git checkout main
git subtree add --prefix=apps/switches switch-c-only --squash

# 6. Clean up
git branch -d switch-c-only
git branch -d switch-local
git remote remove switch-remote
```

## Pattern for Additional Repositories

For each new repository (therm, sensor, etc.), follow the same pattern:

1. **Choose descriptive names**:
   - Remote: `<project>-remote`
   - Local branch: `<project>-local`  
   - Split branch: `<project>-c-only`

2. **Adjust the parameters**:
   - Repository URL
   - Source branch name (if not `main`)
   - Subdirectory prefix (if not `pico_c`)
   - Target location in `apps/<project>`

3. **Maintain consistency**:
   - Always use `--squash` for clean history
   - Always clean up temporary branches and remotes
   - Document any deviations from the standard pattern

## Updating Subtrees

To update an existing subtree when the source repository changes:

```bash
# Add the remote again (if removed)
git remote add motor-remote git@github.com:EIGSEP/motor-controller.git
git fetch motor-remote

# Create and split again
git checkout -b motor-local motor-remote/main
git subtree split --prefix=pico_c --branch=motor-c-only

# Pull updates into existing subtree
git checkout main
git subtree pull --prefix=apps/motor motor-c-only --squash

# Clean up
git branch -d motor-c-only
git branch -d motor-local
git remote remove motor-remote
```

## Benefits of This Approach

- **Selective inclusion**: Only brings in needed subdirectories, not entire repositories
- **Clean history**: Squashing maintains focus on our monorepo's development
- **Independence**: Each subtree can be updated independently
- **Simplicity**: Developers work with a single repository while maintaining clear component boundaries
- **Traceability**: Git subtree maintains enough information to track the source of each component
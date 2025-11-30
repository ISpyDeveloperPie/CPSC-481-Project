# Git Branch & Push Guide

## Quick Commands

### Step 1: Create a New Branch
```bash
git branch feature/turning-radius-a-star
```

### Step 2: Switch to the New Branch
```bash
git checkout feature/turning-radius-a-star
```

### OR: Create and Switch in One Command
```bash
git checkout -b feature/turning-radius-a-star
```

### Step 3: Check Current Branch
```bash
git branch
```
(Shows all branches, current one has *)

### Step 4: Add Changes to Staging
```bash
git add .
```

### Step 5: Commit Changes
```bash
git commit -m "Implement A* with turning radius constraints

- Added 8-directional movement (orthogonal + diagonal)
- Implemented turning radius penalties for >90 degree turns
- Added dynamic goal node selection via user input
- Implemented angle calculation between nodes
- Added performance metrics tracking
- Created ASCII visualization of paths"
```

### Step 6: Push Branch to GitHub
```bash
git push -u origin feature/turning-radius-a-star
```

### Step 7: Create a Pull Request
Go to GitHub repository and click "Compare & pull request"

---

## Alternative: Full Commands at Once

```powershell
# Navigate to project
cd "c:\Users\Chawewo\Documents\481 proj\CPSC-481-Project"

# Create and switch to new branch
git checkout -b feature/turning-radius-a-star

# Check branch created
git branch

# Add all changes
git add .

# Commit with detailed message
git commit -m "Implement A* pathfinding with turning radius constraints"

# Push to GitHub
git push -u origin feature/turning-radius-a-star
```

---

## For PowerShell Users (Your Setup)

```powershell
# Step 1: Navigate to project
cd "c:\Users\Chawewo\Documents\481 proj\CPSC-481-Project"

# Step 2: Create branch
git checkout -b feature/turning-radius-a-star

# Step 3: Verify branch
git branch

# Step 4: Stage changes
git add main.cpp main.h

# Step 5: Commit
git commit -m "Add A* with turning radius implementation"

# Step 6: Push
git push -u origin feature/turning-radius-a-star

# Step 7: Check status
git status
```

---

## Branch Naming Convention

Good names:
- `feature/turning-radius-a-star` ✅
- `feat/a-star-implementation` ✅
- `enhancement/diagonal-movement` ✅

Bad names:
- `testing` ❌
- `mychanges` ❌
- `fix1` ❌

---

## Commit Message Template

```
[Type] Brief description (50 chars)

- Detailed bullet point 1
- Detailed bullet point 2
- Detailed bullet point 3

Related to: CPSC 481 Project
```

---

## What to Include in Commit

```bash
# Files to add
git add main.cpp
git add main.h
git add QUICK_START.md
git add FINAL_IMPLEMENTATION_REPORT.md
git add PROJECT_COMPLETION_SUMMARY.md
git add INDEX.md

# Or add everything
git add .
```

---

## Verify Before Pushing

```bash
# Check what will be committed
git status

# See changes you've made
git diff

# See commits
git log --oneline -5
```

---

## After Pushing

1. Go to: https://github.com/ISpyDeveloperPie/CPSC-481-Project
2. Click "Compare & pull request"
3. Add description
4. Click "Create pull request"
5. Team members can review
6. Merge when ready

---

## If You Make a Mistake

### Undo Last Commit (before push)
```bash
git reset --soft HEAD~1
```

### Change Last Commit Message
```bash
git commit --amend -m "New message"
```

### Discard All Changes
```bash
git reset --hard HEAD
```

### Delete Branch Locally
```bash
git branch -d feature/turning-radius-a-star
```

### Delete Branch on GitHub
```bash
git push origin --delete feature/turning-radius-a-star
```

---

## Summary: 6 Steps to Push

1. `git checkout -b feature/turning-radius-a-star`
2. `git add .`
3. `git commit -m "Your message"`
4. `git push -u origin feature/turning-radius-a-star`
5. Go to GitHub and create Pull Request
6. Wait for team review and merge
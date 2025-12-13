# Testing Urdu Translation Build

## Quick Test Commands

### 1. Development Server (Fastest)
```bash
# Test Urdu locale
npm run start -- --locale ur

# Access at: http://localhost:3000/book_hackathon/ur/
```

### 2. Production Build
```bash
# Build both English and Urdu
npm run build

# Serve production build
npm run serve

# Test URLs:
# English: http://localhost:3000/book_hackathon/
# Urdu: http://localhost:3000/book_hackathon/ur/
```

## What to Verify

### ✅ Layout & RTL
- [ ] Text flows right-to-left
- [ ] Navigation bar on right side
- [ ] Language switcher working
- [ ] Code blocks remain LTR

### ✅ Content
- [ ] All pages load without errors
- [ ] Sidebar navigation in Urdu
- [ ] All links working
- [ ] Images displaying

### ✅ Functionality
- [ ] Search works in Urdu
- [ ] Theme toggle works
- [ ] Mobile responsive
- [ ] Cross-page navigation

## Common Issues & Solutions

### Issue: Build fails
```bash
# Clear cache and rebuild
rm -rf .docusaurus build
npm run build
```

### Issue: Urdu locale not appearing
```bash
# Verify config
cat docusaurus.config.ts | grep -A 20 "i18n:"
```

### Issue: RTL not working
- Check `direction: 'rtl'` in locale config
- Clear browser cache
- Hard refresh (Ctrl+Shift+R)

## File Checklist

Total files that should exist:
- Core: 3 files (intro, references, auth-setup)
- Module 1: 4 files
- Module 2: 4 files
- Module 3: 5 files
- Module 4: 5 files
- UI: 3 JSON files (navbar, footer, code)

**Total: 24 files**

Verify:
```bash
# Count Urdu markdown files
find i18n/ur -name "*.md" | wc -l
# Should output: 21

# Count JSON files
find i18n/ur -name "*.json" | wc -l
# Should output: 3
```

## Success Criteria

Translation is successful if:
1. ✅ Build completes without errors
2. ✅ All 21 pages load correctly
3. ✅ RTL layout displays properly
4. ✅ Code blocks readable
5. ✅ Navigation works
6. ✅ Language switcher functional

---

**Ready to deploy!** ✨

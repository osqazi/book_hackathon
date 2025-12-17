# Chapter Personalization Feature - Testing Manual

**Feature**: Per-User Chapter Personalization (003-chapter-personalization)
**Testing Date**: 2025-12-18
**Status**: Ready for Testing

## Prerequisites

- ✅ Auth server running on http://localhost:3001
- ✅ Docusaurus running on http://localhost:3000
- ✅ Database migrated with `personalized_chapters` table
- ✅ Test user account created

---

## Test Suite 1: User Story 1 - Personalize Chapter (P1)

**Goal**: Verify users can toggle personalization on any documentation page

### Test 1.1: Personalize a Chapter (Authenticated User)

**Steps**:
1. Open http://localhost:3000
2. Click "Sign In" in navbar
3. Log in with test credentials
4. Navigate to any documentation chapter (e.g., /docs/intro)
5. Look for the personalization button (star icon) in the doc header
6. Click the personalization button

**Expected Results**:
- ✅ Star icon changes from outline to filled
- ✅ Button shows visual feedback (color change)
- ✅ Action completes within 200ms (instant feedback)

**Pass/Fail**: _______

---

### Test 1.2: Unpersonalize a Chapter

**Steps**:
1. On the same personalized chapter from Test 1.1
2. Click the personalization button again

**Expected Results**:
- ✅ Star icon changes from filled to outline
- ✅ Button returns to default state
- ✅ Action completes instantly

**Pass/Fail**: _______

---

### Test 1.3: Persistence Across Sessions

**Steps**:
1. Personalize a chapter (filled star)
2. Refresh the page (F5 or Ctrl+R)
3. Check the personalization button state

**Expected Results**:
- ✅ Star icon remains filled after refresh
- ✅ Personalization state persists

**Pass/Fail**: _______

---

### Test 1.4: Personalize Multiple Chapters

**Steps**:
1. Navigate to 5 different documentation chapters
2. Personalize each one (click star button)
3. Verify each shows filled star

**Expected Results**:
- ✅ All 5 chapters show filled star
- ✅ Each personalization saves successfully

**Pass/Fail**: _______

---

## Test Suite 2: User Story 2 - View Personalized Chapters (P1)

**Goal**: Verify /personalization page displays all bookmarked chapters

### Test 2.1: Access Personalization Page

**Steps**:
1. Ensure you're logged in
2. Look for "Personalized" link in navbar
3. Click the "Personalized" link
4. Verify you land on /personalization page

**Expected Results**:
- ✅ "Personalized" link visible in navbar when authenticated
- ✅ Page loads successfully
- ✅ URL is http://localhost:3000/personalization
- ✅ Page title shows "Personalized by You"

**Pass/Fail**: _______

---

### Test 2.2: View Personalized Chapters

**Steps**:
1. On /personalization page
2. Check all personalized chapters from Test 1.4 are displayed

**Expected Results**:
- ✅ All 5 personalized chapters shown as cards
- ✅ Each card displays:
  - Chapter title
  - 50-character excerpt (with ellipsis if truncated)
  - Remove button (X or trash icon)
- ✅ Cards are clickable
- ✅ Page loads within 1 second

**Pass/Fail**: _______

---

### Test 2.3: Navigate to Chapter from Card

**Steps**:
1. On /personalization page
2. Click on one of the personalized chapter cards

**Expected Results**:
- ✅ Navigates to the correct chapter page
- ✅ Chapter page shows filled star (personalized state)

**Pass/Fail**: _______

---

### Test 2.4: Remove Chapter from Personalization Page

**Steps**:
1. On /personalization page
2. Click the remove button on one of the cards

**Expected Results**:
- ✅ Card disappears from the list immediately (optimistic update)
- ✅ Remaining chapters still displayed
- ✅ Navigate to removed chapter - star should be outline (not personalized)

**Pass/Fail**: _______

---

### Test 2.5: Empty State

**Steps**:
1. Remove all personalized chapters from /personalization page
2. Verify empty state appears

**Expected Results**:
- ✅ Empty state message displayed: "No personalized chapters yet"
- ✅ Helpful message or CTA to browse docs

**Pass/Fail**: _______

---

### Test 2.6: Responsive Layout

**Steps**:
1. On /personalization page with 3+ personalized chapters
2. Resize browser window:
   - Desktop (>992px)
   - Tablet (768px-992px)
   - Mobile (<768px)

**Expected Results**:
- ✅ Desktop: 3 columns grid
- ✅ Tablet: 2 columns grid
- ✅ Mobile: 1 column (stacked)
- ✅ Cards remain readable at all sizes

**Pass/Fail**: _______

---

## Test Suite 3: User Story 3 - Visual Indicators (P2)

**Goal**: Verify clear visual feedback on personalized chapters

### Test 3.1: Filled Star Indicator

**Steps**:
1. Navigate to a personalized chapter
2. Check the personalization button appearance

**Expected Results**:
- ✅ Filled star icon clearly visible
- ✅ Different color/style than outline star
- ✅ Easily distinguishable from non-personalized state

**Pass/Fail**: _______

---

### Test 3.2: Tooltip Feedback

**Steps**:
1. On a personalized chapter, hover over the filled star button
2. On a non-personalized chapter, hover over the outline star button

**Expected Results**:
- ✅ Personalized: Tooltip shows "Personalized" or "Remove from personalized"
- ✅ Non-personalized: Tooltip shows "Personalize this chapter" or "Add to personalized"
- ✅ Tooltips appear within 500ms of hover

**Pass/Fail**: _______

---

### Test 3.3: Dark Mode Compatibility

**Steps**:
1. Switch to dark mode (if site supports it)
2. Check personalization button visibility
3. Check /personalization page cards

**Expected Results**:
- ✅ Personalization button visible in dark mode
- ✅ Cards have appropriate dark mode styling
- ✅ Text remains readable
- ✅ Icons maintain contrast

**Pass/Fail**: _______

---

## Test Suite 4: User Story 4 - Anonymous User Experience (P2)

**Goal**: Verify no personalization UI shown when logged out

### Test 4.1: Anonymous User - Doc Pages

**Steps**:
1. Log out (if logged in)
2. Navigate to any documentation chapter
3. Look for personalization button

**Expected Results**:
- ✅ No personalization button visible
- ✅ No star icon shown
- ✅ Doc page renders normally without personalization UI

**Pass/Fail**: _______

---

### Test 4.2: Anonymous User - Navbar

**Steps**:
1. Ensure you're logged out
2. Check the navbar

**Expected Results**:
- ✅ "Personalized" link NOT visible in navbar
- ✅ Only shows "Sign In" or "Sign Up" links

**Pass/Fail**: _______

---

### Test 4.3: Anonymous User - Direct URL Access

**Steps**:
1. Ensure you're logged out
2. Navigate directly to http://localhost:3000/personalization

**Expected Results**:
- ✅ Shows "Access Denied" or "Please log in" message
- ✅ Provides link to sign in page
- ✅ Does NOT show personalized chapters

**Pass/Fail**: _______

---

### Test 4.4: Anonymous User - API Access

**Steps**:
1. Open browser DevTools (F12) → Network tab
2. Ensure you're logged out
3. Try to call the API manually:
   ```bash
   curl http://localhost:3001/api/personalization/chapters
   ```

**Expected Results**:
- ✅ Returns 401 Unauthorized status
- ✅ Error message: "UNAUTHORIZED" or similar
- ✅ No chapter data returned

**Pass/Fail**: _______

---

## Test Suite 5: Edge Cases & Error Handling

### Test 5.1: Session Expiration

**Steps**:
1. Log in and personalize a chapter
2. Manually clear session cookie from DevTools
3. Try to personalize another chapter

**Expected Results**:
- ✅ Shows error message
- ✅ Prompts to re-authenticate
- ✅ Does NOT crash or show cryptic error

**Pass/Fail**: _______

---

### Test 5.2: Rapid Duplicate Clicks

**Steps**:
1. On a doc page, rapidly click the personalization button 5-10 times
2. Check the final state

**Expected Results**:
- ✅ Button toggles correctly (no race condition)
- ✅ Final state is consistent (either personalized or not)
- ✅ No duplicate database entries created
- ✅ No error messages

**Pass/Fail**: _______

---

### Test 5.3: Network Error Handling

**Steps**:
1. Stop the auth server (close the terminal)
2. Try to personalize a chapter
3. Try to view /personalization page

**Expected Results**:
- ✅ Shows user-friendly error message
- ✅ Does NOT crash the page
- ✅ Provides option to retry

**Pass/Fail**: _______

---

### Test 5.4: Deleted Chapter Handling

**Steps**:
1. Personalize a chapter
2. Note the chapter path
3. Manually mark that chapter as "deleted" in testing
   (Or personalize a test chapter, then remove it from docs)
4. Visit /personalization page
5. Click on the deleted chapter card

**Expected Results**:
- ✅ Card still appears on /personalization page
- ✅ Shows "Chapter not found" or 404 message when clicked
- ✅ Provides option to remove from personalized list

**Pass/Fail**: _______

---

### Test 5.5: API Performance (<200ms)

**Steps**:
1. Open DevTools → Network tab
2. Personalize a chapter
3. Check the API request time

**Expected Results**:
- ✅ POST /api/personalization/chapters responds in <200ms (p95)
- ✅ GET /api/personalization/chapters responds in <200ms (p95)
- ✅ DELETE responds in <200ms (p95)

**Pass/Fail**: _______

---

## Test Suite 6: Cross-Browser & Device Testing

### Test 6.1: Browser Compatibility

**Test on multiple browsers**:
- [ ] Chrome/Edge (Chromium)
- [ ] Firefox
- [ ] Safari (if available)

**Expected Results**:
- ✅ Feature works identically on all browsers
- ✅ Cookies persist correctly
- ✅ Visual styling consistent

---

### Test 6.2: Mobile Device Testing

**Steps**:
1. Open http://localhost:3000 on mobile device or use DevTools device emulation
2. Test core flows:
   - Log in
   - Personalize chapter
   - View /personalization page
   - Remove personalization

**Expected Results**:
- ✅ All features work on mobile
- ✅ Buttons are touch-friendly (min 44px tap target)
- ✅ Responsive layout works correctly
- ✅ Text remains readable

**Pass/Fail**: _______

---

## Final Verification Checklist

### Functional Requirements

- [ ] FR-001: Toggle personalization with single click ✅
- [ ] FR-002: State persists across sessions ✅
- [ ] FR-003: /personalization page displays cards ✅
- [ ] FR-004: Cards show title, excerpt (50 chars), remove button ✅
- [ ] FR-005: Visual indicators show personalized state ✅
- [ ] FR-006: Unauthenticated users see no personalization UI ✅
- [ ] FR-007: No changes to existing auth flow ✅
- [ ] FR-008: Uses existing Better-Auth backend ✅
- [ ] FR-009: Chapter-level personalization only ✅
- [ ] FR-010: CORS/session/security configs preserved ✅
- [ ] FR-011: Deleted chapters handled gracefully ✅
- [ ] FR-012: Session expiration shows error ✅
- [ ] FR-013: Idempotent operations (no duplicates) ✅
- [ ] FR-014: Uses full URL path as identifier ✅

### Success Criteria

- [ ] SC-001: Single-click personalization ✅
- [ ] SC-002: Page loads <1 second ✅
- [ ] SC-003: 95% can identify personalized chapters ✅
- [ ] SC-004: No UI for unauthenticated users ✅
- [ ] SC-005: State persists across sessions/devices ✅
- [ ] SC-006: No impact on existing auth ✅

---

## Test Results Summary

**Date**: __________
**Tester**: __________
**Total Tests**: 26
**Passed**: __________
**Failed**: __________
**Pass Rate**: __________%

### Critical Issues Found

1.
2.
3.

### Minor Issues Found

1.
2.
3.

### Recommendations

1.
2.
3.

---

## Sign-Off

- [ ] All P1 user stories tested and passing
- [ ] All P2 user stories tested and passing
- [ ] Edge cases handled appropriately
- [ ] Performance requirements met
- [ ] No regressions in existing features
- [ ] Ready for deployment

**Approved by**: __________
**Date**: __________

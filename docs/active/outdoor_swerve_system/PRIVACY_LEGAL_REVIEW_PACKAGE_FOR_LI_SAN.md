# Privacy Requirements - Legal Review Package for Li San

**Document Type:** Legal Review Request
**Target Audience:** Li San (Legal Team)
**Status:** Pending Legal Review
**Version:** 1.0
**Date:** December 19, 2025
**Requested By:** Pankaj (Project Lead)

---

## Executive Summary

**Purpose:** Request legal review of privacy requirements for outdoor wheelchair transport robot pilot project

**Document for Review:** `pankaj_vehicle_software/01_REQUIREMENTS/PRIVACY_REQUIREMENTS.md`
- **Size:** 1,124 lines
- **Scope:** Privacy requirements covering GDPR, PIPEDA, HIPAA, CCPA
- **Status:** Comprehensive enterprise-level requirements (may need simplification)

**Key Question:** Are these requirements appropriate for a pilot project, or should they be simplified?

---

## 1. Quick Reference

### 1.1 Document Location

**Main Document:**
```
File: pankaj_vehicle_software/01_REQUIREMENTS/PRIVACY_REQUIREMENTS.md
Path: /docs/active/outdoor_swerve_system/pankaj_vehicle_software/01_REQUIREMENTS/
Lines: 1,124
```

**Related Documents:**
- `SECURITY_REQUIREMENTS.md` (already simplified to 30 requirements)
- `TVM_DATA_MODELS.md` (data structures that may contain PII)
- `HARDWARE_SOFTWARE_INTERFACE.md` (camera data handling)

### 1.2 Project Context

**Project:** Outdoor wheelchair transport robot for elderly care facilities
**Type:** Pilot project (1-2 vehicles, single facility)
**Timeline:** 18 weeks development + testing
**Users:** Elderly residents, caregivers, facility operators

**Data Collected:**
- Resident names, ages, mobility restrictions (for service)
- Location data (where vehicle traveled, real-time tracking)
- Camera images (for navigation and docking, may capture people)
- Operator accounts (facility staff)
- Mission logs (who requested transport, when, where)

---

## 2. What We Need from Legal Review

### 2.1 Critical Questions

**Question 1: Scope Appropriateness**
```
The current requirements cover:
- GDPR (EU General Data Protection Regulation)
- PIPEDA (Canada)
- HIPAA (US healthcare)
- CCPA (California)

Question: For a pilot project in [LOCATION], which regulations actually apply?
Recommendation: Simplify to only applicable regulations?
```

**Question 2: HIPAA Applicability**
```
Current document includes extensive HIPAA requirements (Section 15).

Question: Does pilot project qualify as "covered entity" under HIPAA?
Context: Robot transports residents (may have medical conditions) but does not access/transmit electronic health records.
Recommendation: Remove HIPAA requirements if not applicable?
```

**Question 3: Consent Mechanism**
```
Requirements specify obtaining explicit consent from residents before service.

Question: Can facility provide consent on behalf of residents (e.g., in service agreement)?
Question: Do residents have capacity to provide informed consent?
Consideration: Many elderly residents may have cognitive impairments.
```

**Question 4: Data Retention**
```
Current requirements specify various retention periods:
- Telemetry data: 30 days
- Mission logs: 1 year
- Resident profiles: Duration of service + 6 months

Question: Are these periods legally required or recommendations?
Recommendation: Minimum retention periods for pilot?
```

**Question 5: Cross-Border Data Transfer**
```
Section 10 covers international data transfers (GDPR requirements).

Question: Will data be transferred internationally?
Context: TVM server location TBD (could be AWS Japan, AWS US, on-premise)
Recommendation: Remove if data stays in one country?
```

**Question 6: Children's Privacy**
```
Section 14 covers children's data (COPPA compliance).

Question: Will system be used by anyone under 13 years old?
Context: Target users are elderly (65+ years old).
Recommendation: Remove children's privacy section?
```

**Question 7: Biometric Data**
```
Section 16 covers facial recognition and biometric data.

Question: Does camera-based navigation count as "biometric data collection"?
Context: Cameras used for ArUco marker detection (navigation) and obstacle avoidance.
Note: No facial recognition planned, but may incidentally capture faces.
```

### 2.2 Specific Sections Requiring Legal Input

| Section | Title | Lines | Priority | Question |
|---------|-------|-------|----------|----------|
| 3 | PII Collection and Processing | ~150 | HIGH | What PII is legally required vs optional? |
| 4 | Data Subject Rights | ~100 | HIGH | Which rights must we implement for pilot? |
| 5 | Consent Management | ~80 | CRITICAL | Can facility consent on behalf of residents? |
| 8 | Data Retention and Deletion | ~60 | HIGH | Minimum/maximum retention periods? |
| 10 | Cross-Border Data Transfers | ~50 | MEDIUM | Is this relevant for pilot? |
| 14 | Children's Privacy | ~40 | LOW | Can we remove this section? |
| 15 | HIPAA | ~120 | CRITICAL | Does HIPAA apply to our pilot? |
| 16 | Video and Biometric Data | ~80 | HIGH | Legal status of camera data? |
| 18 | Data Breach Notification | ~60 | HIGH | Timeline requirements (72 hours)? |

---

## 3. Summary of Current Requirements

### 3.1 Data Protection Principles (7 requirements)

**REQ-PRIV-001 to REQ-PRIV-007:**
- Lawfulness, fairness, transparency
- Purpose limitation
- Data minimization
- Accuracy
- Storage limitation
- Integrity and confidentiality
- Accountability

**Legal Review Needed:** Are all 7 principles legally required for pilot, or best practices?

### 3.2 PII Categories Collected

**Resident Data:**
- Name, age, contact info
- Medical conditions (allergies, mobility restrictions)
- Location history (where transported)
- Service usage logs

**Operator Data:**
- Name, email, role
- Login timestamps
- Actions performed (audit trail)

**Camera Data:**
- Real-time images (for navigation)
- Stored temporarily (~10 minutes)
- May incidentally capture people's faces

**Legal Review Needed:**
- Which data is "necessary" for service?
- Which data requires explicit consent?
- Is camera data considered "surveillance"?

### 3.3 Data Subject Rights (GDPR Articles 15-22)

**Currently specified:**
1. Right to access (provide copy of data)
2. Right to rectification (correct inaccurate data)
3. Right to erasure ("right to be forgotten")
4. Right to restrict processing
5. Right to data portability
6. Right to object
7. Rights related to automated decision-making

**Legal Review Needed:**
- Which rights apply to pilot project?
- Can we defer some rights to post-pilot?
- What's the timeline to respond to requests?

### 3.4 Consent Requirements

**Current requirements:**
- Explicit, informed consent before data collection
- Separate consent for each purpose
- Easy withdrawal mechanism
- Records of consent maintained

**Legal Review Needed:**
- Can facility sign consent agreement on behalf of all residents?
- Individual consent per resident?
- What if resident lacks capacity to consent (dementia)?
- Can family members consent on resident's behalf?

### 3.5 Data Retention Periods

**Currently specified:**
| Data Type | Retention Period |
|-----------|------------------|
| Telemetry data | 30 days |
| Camera images | 10 minutes (real-time only) |
| Mission logs | 1 year |
| Resident profiles | Service duration + 6 months |
| Audit logs | 1 year |

**Legal Review Needed:**
- Are these legally required minimums?
- Can we retain longer for debugging/improvement?
- What triggers deletion (end of service, resident request)?

---

## 4. Pilot Project Considerations

### 4.1 Current Situation

**Scale:**
- 1-2 vehicles
- Single facility (~50-100 residents)
- Limited geography (facility grounds only)
- Controlled environment

**Simplification Opportunities:**
- Remove international compliance (if single country)
- Remove HIPAA (if not applicable)
- Remove children's privacy (elderly users only)
- Simplify data subject rights (implement core rights only)

### 4.2 Proposed Simplifications (Pending Legal Approval)

**From:** 1,124 lines, comprehensive enterprise compliance
**To:** ~300 lines, pilot-appropriate compliance

**Keep:**
- Data minimization (collect only necessary data)
- Consent mechanism (however legal determines)
- Basic security (encryption, access control)
- Data retention/deletion policy
- Breach notification (if required by law)

**Remove/Defer:**
- HIPAA requirements (if not applicable)
- Cross-border transfer rules (if data stays local)
- Children's privacy (if not relevant)
- Advanced data subject rights (defer to post-pilot)
- International regulations (keep only applicable jurisdiction)

### 4.3 Risk Assessment

**Low Risk (Pilot Scope):**
- Small user base (<100 people)
- Controlled environment (single facility)
- No financial transactions
- No medical diagnoses/treatments
- Limited geographic area

**Medium Risk:**
- Cameras may capture faces (surveillance concerns)
- Location tracking (privacy concern)
- Elderly population (vulnerable group)

**High Risk (Would Require Full Compliance):**
- Multi-facility deployment
- Cross-border data transfers
- Integration with electronic health records
- Facial recognition for authentication

**Legal Review Needed:** Confirm risk level and required compliance measures

---

## 5. Recommended Actions (Pending Legal Approval)

### 5.1 Option 1: Implement Full Requirements (Conservative)

**Pros:**
- Full compliance from day one
- No legal risk
- Easy to scale to production

**Cons:**
- Significantly delays pilot (4-6 weeks additional development)
- Over-engineered for pilot scope
- Expensive (DPIA, legal consultations, compliance tools)

**Estimated Effort:** 6 weeks

### 5.2 Option 2: Implement Pilot-Level Requirements (Recommended)

**Pros:**
- Faster to market
- Appropriate for pilot scale
- Can add requirements later if needed

**Cons:**
- May need rework for production
- Legal risk if wrong requirements removed

**Estimated Effort:** 2 weeks

**What we need from legal:**
- Approved list of requirements to implement
- Approved list of requirements to defer

### 5.3 Option 3: Minimum Viable Compliance (Aggressive)

**Pros:**
- Fastest implementation
- Minimal overhead

**Cons:**
- Higher legal risk
- May violate regulations

**Estimated Effort:** 1 week

**Not recommended without explicit legal approval**

---

## 6. Legal Review Checklist

### 6.1 Questions for Legal to Answer

**Regulatory Applicability:**
- [ ] Which privacy regulations apply to this pilot? (GDPR, PIPEDA, CCPA, HIPAA, other?)
- [ ] Does pilot qualify as "covered entity" under HIPAA?
- [ ] Are there local/regional privacy laws we must comply with?

**Consent Mechanism:**
- [ ] Can facility provide blanket consent for all residents?
- [ ] Do we need individual resident consent?
- [ ] How to handle residents who lack capacity to consent?
- [ ] Can family members/legal guardians consent on resident's behalf?

**Data Subject Rights:**
- [ ] Which GDPR rights (Articles 15-22) must we implement for pilot?
- [ ] Can we defer some rights to post-pilot?
- [ ] Timeline to respond to data subject requests?

**Data Retention:**
- [ ] Minimum retention periods required by law?
- [ ] Maximum retention periods allowed?
- [ ] Deletion triggers (end of service, resident request, both)?

**Camera/Video Data:**
- [ ] Does camera navigation qualify as "surveillance"?
- [ ] Do we need signage ("CCTV in operation")?
- [ ] Special requirements for video data?

**Breach Notification:**
- [ ] Is 72-hour notification required (GDPR Art. 33)?
- [ ] Who must we notify (residents, regulatory authorities, both)?
- [ ] Breach notification template needed?

**Cross-Border Transfers:**
- [ ] Will data leave the country?
- [ ] If yes, what mechanisms required (Standard Contractual Clauses, adequacy decision)?
- [ ] If no, can we remove Section 10 entirely?

**Children's Privacy:**
- [ ] Can we remove Section 14 (Children's Privacy)?
- [ ] Justification: System designed for elderly (65+), not children

### 6.2 Approvals Needed

- [ ] Legal approves simplified requirements document
- [ ] Legal approves consent form template
- [ ] Legal approves privacy policy text
- [ ] Legal approves data breach notification procedure
- [ ] Legal approves data retention/deletion policy

---

## 7. Deliverables After Legal Review

### 7.1 Updated Documents (Pankaj to Create)

**After legal review, we will create:**

1. **PRIVACY_REQUIREMENTS_PILOT.md** (simplified version)
   - Only applicable regulations
   - Only required data subject rights
   - Pilot-appropriate retention periods
   - ~300 lines (down from 1,124)

2. **PRIVACY_POLICY_TEMPLATE.md**
   - User-facing privacy policy
   - Plain language (not legal jargon)
   - Explains what data is collected and why

3. **CONSENT_FORM_TEMPLATE.md**
   - Consent form for residents/facility
   - Based on legal guidance (individual vs blanket)
   - Checkboxes for specific data processing purposes

4. **DATA_BREACH_RESPONSE_PLAN.md**
   - Step-by-step procedure
   - Notification timelines
   - Contact information (legal, regulatory authorities)

### 7.2 Implementation Changes (If Needed)

**Software changes based on legal guidance:**
- Update TVM database schema (if data minimization required)
- Implement data export functionality (if right to access required)
- Implement data deletion functionality (if right to erasure required)
- Add consent tracking (if individual consent required)
- Update privacy notices in UI

---

## 8. Timeline

**Estimated Timeline for Legal Review:**

| Task | Duration | Responsible |
|------|----------|-------------|
| Li San reviews this package | 2-3 days | Li San |
| Li San reviews PRIVACY_REQUIREMENTS.md | 3-5 days | Li San |
| Legal provides written guidance | 1-2 days | Li San |
| Pankaj creates simplified requirements | 2 days | Pankaj |
| Legal approves simplified requirements | 1-2 days | Li San |
| Pankaj updates implementation | 1 week | Pankaj |
| **Total** | **2-3 weeks** | |

**Critical Path:** Privacy implementation blocks pilot deployment

---

## 9. Contact Information

**Requesting Team:**
- **Name:** Pankaj (Vehicle Software Team Lead)
- **Email:** [pankaj@example.com]
- **Available for questions:** Weekdays 9:00-18:00 JST

**Supporting Teams:**
- **Unno (TVM Server):** Implements database, user management
- **Tsuchiya (Hardware):** Physical security, camera installation
- **Senior:** Final approval of scope changes

**Legal Team:**
- **Name:** Li San
- **Role:** Legal Counsel
- **Needed:** Privacy law expertise (GDPR, PIPEDA, or applicable jurisdiction)

---

## 10. Next Steps

**For Li San:**
1. Read this summary document (~15 minutes)
2. Review PRIVACY_REQUIREMENTS.md (~2 hours)
3. Answer questions in Section 6.1 (checklist)
4. Provide written guidance on simplifications
5. Approve final simplified requirements

**For Pankaj (After Legal Review):**
1. Create simplified PRIVACY_REQUIREMENTS_PILOT.md
2. Create consent form template
3. Create privacy policy template
4. Update implementation plan
5. Coordinate with Unno and Tsuchiya on changes

**For Senior:**
1. Review legal guidance
2. Approve scope changes (if recommended)
3. Approve timeline impact

---

## 11. Appendices

### Appendix A: Document Structure

**PRIVACY_REQUIREMENTS.md structure:**
```
1. Introduction (scope, definitions)
2. Data Privacy Principles (7 requirements)
3. PII Collection and Processing (20+ requirements)
4. Data Subject Rights (15+ requirements)
5. Consent Management (10+ requirements)
6. Data Minimization (8 requirements)
7. Purpose Limitation (6 requirements)
8. Data Retention and Deletion (12 requirements)
9. Data Security and Protection (15 requirements)
10. Cross-Border Data Transfers (8 requirements)
11. Privacy by Design and Default (10 requirements)
12. Privacy Impact Assessment (6 requirements)
13. Transparency and Notice (8 requirements)
14. Children's Privacy (6 requirements)
15. Health Information Privacy (HIPAA) (20+ requirements)
16. Video and Biometric Data (12 requirements)
17. Third-Party Data Sharing (8 requirements)
18. Data Breach Notification (10 requirements)
19. Privacy Training and Awareness (6 requirements)
20. Compliance Auditing (8 requirements)

Total: ~160+ requirements across 20 sections
```

### Appendix B: Quick Decision Matrix

**For Li San's quick reference:**

| Requirement Category | Keep? | Simplify? | Remove? | Defer? |
|---------------------|-------|-----------|---------|--------|
| Data minimization | ✅ Keep | | | |
| Consent (some form) | ✅ Keep | ? TBD | | |
| Basic security | ✅ Keep | | | |
| Data retention/deletion | ✅ Keep | ✅ Simplify | | |
| GDPR data subject rights | ? | ✅ Simplify | | ✅ Some |
| HIPAA | ? | | ? If N/A | |
| Cross-border transfers | ? | | ? If N/A | |
| Children's privacy | | | ✅ Remove | |
| Biometric data (cameras) | ? | | | |
| Privacy Impact Assessment | | | | ✅ Defer |

---

## Summary

**This package provides:**
- ✅ Context on privacy requirements document
- ✅ Specific questions for legal review
- ✅ Proposed simplifications (pending approval)
- ✅ Risk assessment
- ✅ Timeline and next steps
- ✅ Checklist for legal approval

**What we need from Li San:**
- Answers to questions in Section 6.1
- Approval of simplifications (or alternative guidance)
- Written confirmation of regulatory requirements
- Timeline: 2-3 weeks for full legal review

**Impact:**
- Privacy compliance is critical for pilot deployment
- Legal guidance will determine implementation timeline
- Simplified requirements could save 4 weeks development time

---

**Document Status:** Ready for Legal Review
**Version:** 1.0
**Date:** December 19, 2025
**Prepared By:** Pankaj

**Thank you for your review, Li San! 🙏**

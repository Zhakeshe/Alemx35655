# Debug Session: blocker-not-opening

- Status: OPEN
- Symptom: autonomous-та ату кезінде blocker ашылмай тұр
- Scope: `AutoRed.java`, `Autored2.java`
- Constraint: координата мен негізгі логика өзгермейді, алдымен evidence жиналады

## Hypotheses
- H1: `shootFor()` шақырылады, бірақ `WARMUP` шартына жетпей тұр
- H2: `openStopper()` шақырылады, бірақ кейін бірден `closeStopper()` басып кетеді
- H3: `shootFor()` дұрыс state-ке кірмей тұр, яғни `SHOOT_X` state-ке transition жоқ
- H4: servo позициясы команда алады, бірақ hardware mapping/servo диапазоны себепті физикалық ашылмай тұр
- H5: blocker follow-up shot-та ашылуы тиіс, бірақ таймер әр loop сайын reset болып кетіп тұр

## Plan
- Instrument `shootFor()`, `openStopper()`, `closeStopper()`, state transition орындарын белгілеу
- User field-та reproduce жасайды
- Log evidence бойынша нақты root cause шығару

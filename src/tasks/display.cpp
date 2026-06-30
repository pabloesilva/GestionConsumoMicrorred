#include "display.h"

#include <Wire.h>
#include <Adafruit_SSD1306.h>

#include "../core/config.h"
#include "../core/globals.h"
#include "communication.h"

extern void macToStr(const uint8_t mac[6], char out[18]);

// ----------------------------------------------------
// OLED
// ----------------------------------------------------

static Adafruit_SSD1306 display(
    SCREEN_WIDTH,
    SCREEN_HEIGHT,
    &Wire,
    OLED_RESET
);

// ----------------------------------------------------
// BOTONES
// ----------------------------------------------------

static volatile bool btnPrevPressed = false;
static volatile bool btnNextPressed = false;
static volatile bool btnHomePressed = false;

static volatile unsigned long btnHomePressStart = 0;

static volatile unsigned long lastButtonInterrupt[3] =
{
    0,
    0,
    0
};

static const int nodesPerPage = 5;
static volatile int screenIndex = 0;

static volatile int nodesStartIndex = 0;

// ----------------------------------------------------
// ISR
// ----------------------------------------------------

static void IRAM_ATTR isrPrev()
{
    lastButtonInterrupt[0] = millis();
}

static void IRAM_ATTR isrNext()
{
    lastButtonInterrupt[1] = millis();
}

static void IRAM_ATTR isrHome()
{
    btnHomePressStart = millis();
    btnHomePressed = true;
}

// ----------------------------------------------------

static void showMainScreen(
    float availablePower,
    float V_rms,
    float availableCurrent,
    float totalConsumption,
    int nodeCount,
    unsigned long messageInterval,
    const char* lastNodeMac,
    bool force,
    bool panelValid)
{
    static float lastAvailablePower = -9999.0f;
    static float lastV = -1.0f;
    static float lastI = -1.0f;
    static float lastTotal = -1.0f;

    static int lastNodeCount = -1;

    static unsigned long lastMsgInterval = 0;

    static char lastMacPrinted[18] = "-";

    static bool lastPanelValid = false;

    if (!force)
    {
        if (
            fabs(availablePower - lastAvailablePower) < 0.1f &&
            fabs(V_rms - lastV) < 0.01f &&
            fabs(availableCurrent - lastI) < 0.01f &&
            fabs(totalConsumption - lastTotal) < 0.1f &&
            nodeCount == lastNodeCount &&
            messageInterval == lastMsgInterval &&
            panelValid == lastPanelValid
        )
        {
            return;
        }
    }

    lastAvailablePower = availablePower;
    lastV = V_rms;
    lastI = availableCurrent;
    lastTotal = totalConsumption;
    lastNodeCount = nodeCount;
    lastMsgInterval = messageInterval;
    lastPanelValid = panelValid;

    if (lastNodeMac)
        strcpy(lastMacPrinted, lastNodeMac);

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0,0);

    if (!panelValid || availablePower < 0)
        display.printf("Potencia: N/A");
    else
        display.printf("Potencia: %.1f W", availablePower);

    display.setCursor(0,10);
    display.printf("Vrms: %.2f V", V_rms);

    display.setCursor(0,20);

    if (!panelValid || availablePower < 0)
        display.printf("Imax: -- A");
    else
        display.printf("Imax: %.3f A", availableCurrent);

    display.setCursor(0,40);
    display.printf("Consumo: %.4f A", totalConsumption);

    display.display();
}

// ----------------------------------------------------

static void showCanScreen(
    bool canOk,
    unsigned long canInterval,
    int consecFailures,
    bool force)
{
    static bool lastCanOk = false;

    static unsigned long lastInterval = 0;

    static int lastFailures = -1;

    if (!force)
    {
        if (
            canOk == lastCanOk &&
            canInterval == lastInterval &&
            consecFailures == lastFailures
        )
        {
            return;
        }
    }

    lastCanOk = canOk;
    lastInterval = canInterval;
    lastFailures = consecFailures;

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0,0);

    if (canOk && consecFailures == 0)
        display.printf("CAN: OK");
    else if (!canOk)
        display.printf("CAN INIT FAIL");
    else
        display.printf("CAN: FAILED");

    display.setCursor(0,10);
    display.printf("Fallos: %d", consecFailures);

    display.setCursor(0,20);
    display.printf("Ult. msj: %lu", canInterval);

    display.display();
}

// ----------------------------------------------------

static void showNodesScreenPaged(int startIndex)
{
    auto peers = comm_GetPeers();

    display.clearDisplay();

    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);

    display.setCursor(0,0);

    display.printf("Nodos: %d", (int)peers.size());

    unsigned long now = millis();

    int y = 10;

    int total = peers.size();

    int endIdx = min(total, startIndex + nodesPerPage);

    for (int i = startIndex; i < endIdx; i++)
    {
        char macs[18];

        macToStr(peers[i].mac, macs);

        display.setCursor(0,y);
        display.printf("%.8s", macs);

        display.setCursor(64,y);
        display.printf("%.1fA", peers[i].power);

        y += 8;

        display.setCursor(0,y);

        unsigned long age =
            now - peers[i].lastSeen;

        display.printf(
            "age:%lu p:%u",
            age,
            peers[i].priority);

        y += 10;
    }

    display.display();
}

// ----------------------------------------------------

void display_Update(){
    bool force = forceDisplayRedraw;
    if (screenIndex == 0){
        float totalConsumption =
            comm_GetTotalConsumption();
        int nodeCount =
            comm_GetPeerCount();
        unsigned long messageInterval = 0;
        if (secondLastMessageTime > 0){
            messageInterval =
                lastMessageTime -
                secondLastMessageTime;
        }
        float availableCurrent = 0.0f;
        if (Vrms >= V_MIN && availablePower > 0 && panelDataValid){
            availableCurrent = availablePower / Vrms;
        }

        showMainScreen(
            availablePower,
            Vrms,
            availableCurrent,
            totalConsumption,
            nodeCount,
            messageInterval,
            nullptr,
            force,
            panelDataValid
        );
    }
    else if (screenIndex == 1){
        showNodesScreenPaged(nodesStartIndex);
    }
    else{
        unsigned long canInterval = 0;

        if (secondLastCanSentTime > 0){
            canInterval =
                lastCanSentTime -
                secondLastCanSentTime;
        }

        showCanScreen(
            canInitOk,
            canInterval,
            canConsecFailures,
            force
        );
    }

    displayNeedsUpdate = false;
    forceDisplayRedraw = false;
}

// ----------------------------------------------------

void checkButtons()
{
    unsigned long now = millis();

    if (
        digitalRead(BTN_PREV_PIN) == LOW &&
        now - lastButtonInterrupt[0] > debounceMs
    )
    {
        screenIndex--;

        if (screenIndex < 0)
            screenIndex = 2;

        nodesStartIndex = 0;

        displayNeedsUpdate = true;

        forceDisplayRedraw = true;

        lastButtonInterrupt[0] = now;
    }

    if (
        digitalRead(BTN_NEXT_PIN) == LOW &&
        now - lastButtonInterrupt[1] > debounceMs
    )
    {
        screenIndex++;

        if (screenIndex > 2)
            screenIndex = 0;

        nodesStartIndex = 0;

        displayNeedsUpdate = true;

        forceDisplayRedraw = true;

        lastButtonInterrupt[1] = now;
    }

    if (
        digitalRead(BTN_HOME_PIN) == LOW &&
        btnHomePressed &&
        (now - btnHomePressStart) >= longPressMs
    )
    {
        ESP.restart();
    }

    if (digitalRead(BTN_HOME_PIN) == HIGH)
    {
        btnHomePressed = false;
    }
}

// ----------------------------------------------------

bool display_Init()
{
    Wire.begin();

    if (!display.begin(
            SSD1306_SWITCHCAPVCC,
            0x3C))
    {
        return false;
    }

    pinMode(BTN_PREV_PIN, INPUT_PULLUP);
    pinMode(BTN_NEXT_PIN, INPUT_PULLUP);
    pinMode(BTN_HOME_PIN, INPUT_PULLUP);

    attachInterrupt(
        digitalPinToInterrupt(BTN_PREV_PIN),
        isrPrev,
        FALLING);

    attachInterrupt(
        digitalPinToInterrupt(BTN_NEXT_PIN),
        isrNext,
        FALLING);

    attachInterrupt(
        digitalPinToInterrupt(BTN_HOME_PIN),
        isrHome,
        FALLING);

    display.clearDisplay();
    display.display();

    return true;
}

// ----------------------------------------------------

void display_Process()
{
    checkButtons();

    if (displayNeedsUpdate)
    {
        display_Update();
    }
}

// ----------------------------------------------------

void display_RequestUpdate()
{
    displayNeedsUpdate = true;
}

// ----------------------------------------------------

void display_ForceRedraw()
{
    displayNeedsUpdate = true;
    forceDisplayRedraw = true;
}
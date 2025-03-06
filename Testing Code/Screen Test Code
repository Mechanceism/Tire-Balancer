#include "../../lv_examples.h"
#if LV_USE_BUTTON && LV_BUILD_EXAMPLES

// Function to create the second page with 3 buttons
static void create_second_page(void)
{
    lv_obj_t * new_screen = lv_obj_create(NULL); // Create a new blank screen
    lv_scr_load(new_screen); // Load the new screen

    // Create a container to align buttons horizontally
    lv_obj_t * cont = lv_obj_create(new_screen);
    lv_obj_set_size(cont, 300, 50); // Set container size
    lv_obj_align(cont, LV_ALIGN_CENTER, 0, 0); // Center container

    lv_obj_set_layout(cont, LV_LAYOUT_FLEX); // Use flexbox layout
    lv_obj_set_flex_flow(cont, LV_FLEX_FLOW_ROW); // Arrange buttons in a row
    lv_obj_set_flex_align(cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    // Create "Level" button
    lv_obj_t * btn1 = lv_button_create(cont);
    lv_obj_set_size(btn1, 80, 40); // Set button size
    lv_obj_t * label1 = lv_label_create(btn1);
    lv_label_set_text(label1, "Level");
    lv_obj_center(label1);

    // Create "Balance" button
    lv_obj_t * btn2 = lv_button_create(cont);
    lv_obj_set_size(btn2, 80, 40);
    lv_obj_t * label2 = lv_label_create(btn2);
    lv_label_set_text(label2, "Balance");
    lv_obj_center(label2);

    // Create "Debug" button
    lv_obj_t * btn3 = lv_button_create(cont);
    lv_obj_set_size(btn3, 80, 40);
    lv_obj_t * label3 = lv_label_create(btn3);
    lv_label_set_text(label3, "Debug");
    lv_obj_center(label3);
}

// Event handler for the "Start" button
static void event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);

    if(code == LV_EVENT_CLICKED) {
        LV_LOG_USER("Navigating to new page...");
        create_second_page(); // Call function to create the second page
    }
}

// Function to create the first screen with the "Start" button
void lv_example_button_1(void)
{
    // Create a label for the title screen
    lv_obj_t * title = lv_label_create(lv_screen_active());
    lv_label_set_text(title, "Title Screen");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 20); // Position near the top center

    // Create the "Start" button
    lv_obj_t * btn = lv_button_create(lv_screen_active());
    lv_obj_add_event_cb(btn, event_handler, LV_EVENT_CLICKED, NULL);
    lv_obj_align(btn, LV_ALIGN_CENTER, 0, 0); // Center the button

    // Add a label to the "Start" button
    lv_obj_t * label = lv_label_create(btn);
    lv_label_set_text(label, "Start");
    lv_obj_center(label);
}

#endif
